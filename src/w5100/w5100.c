/*
 * W5100 MAC and PHY driver for esp_eth, written for wifi-bridge along the
 * lines of the ESP-IDF W5500 driver (see ../w5500/w5500_mac.c). Socket 0
 * runs in MACRAW mode, which on the W5100 has no MAC filter: every frame on
 * the wire is received.
 *
 * SPI: the W5100 moves one data byte per 4-byte frame (opcode, address, data)
 * and needs CS raised after every frame, so a full-size Ethernet frame takes
 * ~1500 SPI transactions. Rather than paying the ESP-IDF SPI driver's
 * per-transaction overhead for each, one ESP-IDF transaction sets the SPI
 * peripheral up for 32-bit full-duplex frames and every frame after that is
 * started through the registers directly: 3.5 us per byte at 13.3 MHz, so
 * ~5 ms per full-size Ethernet frame. That needs a bus without DMA (see
 * main.c) that stays acquired by this device.
 *
 * PHY: the W5100 autonegotiates by itself and has no register to read the link
 * state or the negotiated mode, so the PHY driver reports a permanent link.
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <inttypes.h>
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "hal/spi_ll.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "w5100";

#define W5100_REG_MR        0x0000 // Mode
#define W5100_REG_SHAR      0x0009 // Source hardware (MAC) address
#define W5100_REG_IMR       0x0016 // Interrupt mask
#define W5100_REG_RMSR      0x001A // RX memory size per socket
#define W5100_REG_TMSR      0x001B // TX memory size per socket
#define W5100_REG_S0_MR     0x0400 // Socket 0 mode
#define W5100_REG_S0_CR     0x0401 // Socket 0 command
#define W5100_REG_S0_IR     0x0402 // Socket 0 interrupt
#define W5100_REG_S0_SR     0x0403 // Socket 0 status
#define W5100_REG_S0_TX_FSR 0x0420 // Socket 0 TX free size
#define W5100_REG_S0_TX_WR  0x0424 // Socket 0 TX write pointer
#define W5100_REG_S0_RX_RSR 0x0426 // Socket 0 RX received size
#define W5100_REG_S0_RX_RD  0x0428 // Socket 0 RX read pointer

#define W5100_TX_BASE  0x4000
#define W5100_RX_BASE  0x6000
#define W5100_BUF_MASK 0x1FFF  // socket 0 gets all 8 KB of each buffer
#define W5100_MSR_8K   0x03    // RMSR/TMSR value giving socket 0 all 8 KB
#define W5100_MSR_RESET 0x55   // RMSR/TMSR value after reset (2 KB per socket)

#define W5100_OP_WRITE 0xF0
#define W5100_OP_READ  0x0F

#define W5100_MR_RST      (1 << 7) // Software reset
#define W5100_MR_PB       (1 << 4) // Ping block
#define W5100_IMR_S0      (1 << 0) // Socket 0 interrupt
#define W5100_SMR_MACRAW  0x04
#define W5100_SSR_MACRAW  0x42
#define W5100_SCR_OPEN    0x01
#define W5100_SCR_CLOSE   0x10
#define W5100_SCR_SEND    0x20
#define W5100_SCR_RECV    0x40
#define W5100_SIR_SEND_OK (1 << 4)
#define W5100_SIR_RECV    (1 << 2)

#define W5100_LOCK_TIMEOUT_MS 100  // longer than copying a full frame (~5 ms)
#define W5100_CMD_TIMEOUT_MS  100

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    spi_device_handle_t spi;
    spi_dev_t *hw;
    bool bus_acquired;
    SemaphoreHandle_t lock;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t addr[6];
} emac_w5100_t;

// ---------------------------------------------------------------- SPI

// One W5100 SPI frame; the caller holds the lock. The first byte out is the
// lowest byte of data_buf[0]. Writing the register and polling the usr bit
// directly instead of through the HAL's buffer and done-flag helpers took
// 4.0 down to 3.5 us per frame at 13.3 MHz (2.4 us of it is the frame).
static inline uint8_t w5100_xfer(emac_w5100_t *emac, uint8_t op, uint16_t addr, uint8_t data)
{
    spi_dev_t *hw = emac->hw;
    hw->data_buf[0] = op | (addr >> 8) << 8 | (addr & 0xFF) << 16 | (uint32_t)data << 24;
    spi_ll_apply_config(hw);
    spi_ll_user_start(hw);
    while (hw->cmd.usr) {
    }
    return hw->data_buf[0] >> 24;
}

static inline bool w5100_lock(emac_w5100_t *emac)
{
    return xSemaphoreTake(emac->lock, pdMS_TO_TICKS(W5100_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline void w5100_unlock(emac_w5100_t *emac)
{
    xSemaphoreGive(emac->lock);
}

static esp_err_t w5100_write(emac_w5100_t *emac, uint16_t addr, const void *data, uint32_t len)
{
    const uint8_t *p = data;
    ESP_RETURN_ON_FALSE(w5100_lock(emac), ESP_ERR_TIMEOUT, TAG, "SPI lock timeout");
    for (uint32_t i = 0; i < len; i++) {
        w5100_xfer(emac, W5100_OP_WRITE, addr + i, p[i]);
    }
    w5100_unlock(emac);
    return ESP_OK;
}

static esp_err_t w5100_read(emac_w5100_t *emac, uint16_t addr, void *data, uint32_t len)
{
    uint8_t *p = data;
    ESP_RETURN_ON_FALSE(w5100_lock(emac), ESP_ERR_TIMEOUT, TAG, "SPI lock timeout");
    for (uint32_t i = 0; i < len; i++) {
        p[i] = w5100_xfer(emac, W5100_OP_READ, addr + i, 0);
    }
    w5100_unlock(emac);
    return ESP_OK;
}

static esp_err_t w5100_write_u8(emac_w5100_t *emac, uint16_t addr, uint8_t value)
{
    return w5100_write(emac, addr, &value, 1);
}

static esp_err_t w5100_read_u16(emac_w5100_t *emac, uint16_t addr, uint16_t *value)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(w5100_read(emac, addr, b, 2), TAG, "read failed");
    *value = b[0] << 8 | b[1];
    return ESP_OK;
}

static esp_err_t w5100_write_u16(emac_w5100_t *emac, uint16_t addr, uint16_t value)
{
    uint8_t b[2] = {value >> 8, value & 0xFF};
    return w5100_write(emac, addr, b, 2);
}

// The 16-bit size registers can change between reading their two bytes:
// read until two reads agree.
static esp_err_t w5100_read_u16_stable(emac_w5100_t *emac, uint16_t addr, uint16_t *value)
{
    uint16_t v0, v1;
    do {
        ESP_RETURN_ON_ERROR(w5100_read_u16(emac, addr, &v0), TAG, "read failed");
        ESP_RETURN_ON_ERROR(w5100_read_u16(emac, addr, &v1), TAG, "read failed");
    } while (v0 != v1);
    *value = v0;
    return ESP_OK;
}

// Copies to or from a socket buffer starting at pointer `ptr`, wrapping
// around at the end of the buffer.
static esp_err_t w5100_write_buf(emac_w5100_t *emac, uint16_t ptr, const uint8_t *data, uint32_t len)
{
    ESP_RETURN_ON_FALSE(w5100_lock(emac), ESP_ERR_TIMEOUT, TAG, "SPI lock timeout");
    for (uint32_t i = 0; i < len; i++) {
        w5100_xfer(emac, W5100_OP_WRITE, W5100_TX_BASE + ((ptr + i) & W5100_BUF_MASK), data[i]);
    }
    w5100_unlock(emac);
    return ESP_OK;
}

static esp_err_t w5100_read_buf(emac_w5100_t *emac, uint16_t ptr, uint8_t *data, uint32_t len)
{
    ESP_RETURN_ON_FALSE(w5100_lock(emac), ESP_ERR_TIMEOUT, TAG, "SPI lock timeout");
    for (uint32_t i = 0; i < len; i++) {
        data[i] = w5100_xfer(emac, W5100_OP_READ, W5100_RX_BASE + ((ptr + i) & W5100_BUF_MASK), 0);
    }
    w5100_unlock(emac);
    return ESP_OK;
}

// Holds the lock until the W5100 has taken the command, so the RX task's RECV
// and the transmit path's SEND can't overwrite each other.
static esp_err_t w5100_command(emac_w5100_t *emac, uint8_t cmd)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(w5100_lock(emac), ESP_ERR_TIMEOUT, TAG, "SPI lock timeout");
    w5100_xfer(emac, W5100_OP_WRITE, W5100_REG_S0_CR, cmd);
    int64_t start = esp_timer_get_time();
    while (w5100_xfer(emac, W5100_OP_READ, W5100_REG_S0_CR, 0)) {
        int64_t elapsed = esp_timer_get_time() - start;
        if (elapsed > W5100_CMD_TIMEOUT_MS * 1000) {
            ESP_LOGE(TAG, "command 0x%02x timeout", cmd);
            ret = ESP_ERR_TIMEOUT;
            break;
        }
        if (elapsed > 2000) {
            vTaskDelay(1);
        }
    }
    w5100_unlock(emac);
    return ret;
}

static void *w5100_spi_init(emac_w5100_t *emac, const eth_w5500_config_t *config)
{
    spi_device_interface_config_t devcfg = *config->spi_devcfg;
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    // With input_delay_ns 0, ESP-IDF assumes MISO arrives early and delays
    // sampling. On the Arduino Ethernet Shield every byte read then came in
    // shifted right by one bit (0x55 as 0x2a). 25-70 ns gives plain mode-0
    // sampling at 13.3 MHz.
    if (devcfg.input_delay_ns == 0) {
        devcfg.input_delay_ns = 40;
    }
    ESP_RETURN_ON_FALSE(spi_bus_add_device(config->spi_host_id, &devcfg, &emac->spi) == ESP_OK, NULL, TAG, "adding SPI device failed");
    ESP_RETURN_ON_FALSE(spi_device_acquire_bus(emac->spi, portMAX_DELAY) == ESP_OK, NULL, TAG, "acquiring SPI bus failed");
    emac->bus_acquired = true;
    // Leaves the peripheral configured for 32-bit full-duplex frames; w5100_xfer
    // only fills in the data and starts it.
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 32,
        .tx_data = {W5100_OP_READ, W5100_REG_MR >> 8, W5100_REG_MR & 0xFF, 0},
    };
    ESP_RETURN_ON_FALSE(spi_device_polling_transmit(emac->spi, &t) == ESP_OK, NULL, TAG, "SPI transaction failed");
    emac->hw = SPI_LL_GET_HW(config->spi_host_id);
    int khz = 0;
    spi_device_get_actual_freq(emac->spi, &khz);
    ESP_LOGI(TAG, "SPI clock %d kHz", khz);
    return emac->spi;
}

// ---------------------------------------------------------------- MAC

static esp_err_t w5100_reset(emac_w5100_t *emac)
{
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_MR, W5100_MR_RST), TAG, "write MR failed");
    for (uint32_t to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        uint8_t mr;
        ESP_RETURN_ON_ERROR(w5100_read(emac, W5100_REG_MR, &mr, 1), TAG, "read MR failed");
        if (!(mr & W5100_MR_RST)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "reset timeout");
    return ESP_ERR_TIMEOUT;
}

// The W5100 has no version register: check the reset values of the memory
// size registers, then that a written value reads back.
static esp_err_t w5100_verify_id(emac_w5100_t *emac)
{
    uint8_t msr[2], mr;
    ESP_RETURN_ON_ERROR(w5100_read(emac, W5100_REG_RMSR, msr, 2), TAG, "read RMSR failed");
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_MR, W5100_MR_PB), TAG, "write MR failed");
    ESP_RETURN_ON_ERROR(w5100_read(emac, W5100_REG_MR, &mr, 1), TAG, "read MR failed");
    if (msr[0] != W5100_MSR_RESET || msr[1] != W5100_MSR_RESET || mr != W5100_MR_PB) {
        ESP_LOGE(TAG, "no W5100 found (RMSR 0x%02x, TMSR 0x%02x, MR 0x%02x)", msr[0], msr[1], mr);
        return ESP_ERR_INVALID_VERSION;
    }
    return ESP_OK;
}

static esp_err_t w5100_setup_default(emac_w5100_t *emac)
{
    // Only socket 0 can do MACRAW: give it all buffer memory
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_RMSR, W5100_MSR_8K), TAG, "write RMSR failed");
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_TMSR, W5100_MSR_8K), TAG, "write TMSR failed");
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_IMR, 0), TAG, "write IMR failed");
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_S0_MR, W5100_SMR_MACRAW), TAG, "write S0_MR failed");
    return ESP_OK;
}

static esp_err_t emac_w5100_start(esp_eth_mac_t *mac)
{
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    uint8_t sr = 0;
    ESP_RETURN_ON_ERROR(w5100_command(emac, W5100_SCR_OPEN), TAG, "issue OPEN command failed");
    ESP_RETURN_ON_ERROR(w5100_read(emac, W5100_REG_S0_SR, &sr, 1), TAG, "read S0_SR failed");
    ESP_RETURN_ON_FALSE(sr == W5100_SSR_MACRAW, ESP_FAIL, TAG, "socket didn't open in MACRAW mode (status 0x%02x)", sr);
    if (emac->int_gpio_num >= 0) {
        ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_IMR, W5100_IMR_S0), TAG, "write IMR failed");
    }
    return ESP_OK;
}

static esp_err_t emac_w5100_stop(esp_eth_mac_t *mac)
{
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    ESP_RETURN_ON_ERROR(w5100_write_u8(emac, W5100_REG_IMR, 0), TAG, "write IMR failed");
    ESP_RETURN_ON_ERROR(w5100_command(emac, W5100_SCR_CLOSE), TAG, "issue CLOSE command failed");
    return ESP_OK;
}

static esp_err_t emac_w5100_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    ESP_RETURN_ON_FALSE(eth, ESP_ERR_INVALID_ARG, TAG, "can't set mac's mediator to null");
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    emac->eth = eth;
    return ESP_OK;
}

static esp_err_t emac_w5100_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    return ESP_ERR_NOT_SUPPORTED;  // the W5100 has no PHY registers
}

static esp_err_t emac_w5100_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w5100_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    ESP_RETURN_ON_FALSE(addr, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    memcpy(emac->addr, addr, 6);
    return w5100_write(emac, W5100_REG_SHAR, emac->addr, 6);
}

static esp_err_t emac_w5100_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    ESP_RETURN_ON_FALSE(addr, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    memcpy(addr, emac->addr, 6);
    return ESP_OK;
}

static esp_err_t emac_w5100_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        ESP_RETURN_ON_ERROR(mac->start(mac), TAG, "w5100 start failed");
        if (emac->poll_timer) {
            ESP_RETURN_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000), TAG, "start poll timer failed");
        }
        return ESP_OK;
    case ETH_LINK_DOWN:
        ESP_RETURN_ON_ERROR(mac->stop(mac), TAG, "w5100 stop failed");
        if (emac->poll_timer) {
            ESP_RETURN_ON_ERROR(esp_timer_stop(emac->poll_timer), TAG, "stop poll timer failed");
        }
        return ESP_OK;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t emac_w5100_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    return ESP_OK;  // negotiated by the W5100 itself
}

static esp_err_t emac_w5100_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    return ESP_OK;
}

static esp_err_t emac_w5100_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    // MACRAW on the W5100 has no MAC filter, it always receives every frame
    return enable ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w5100_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w5100_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w5100_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    uint16_t free_size, ptr;

    ESP_RETURN_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, TAG,
                        "frame size is too big (actual %" PRIu32 ", maximum %u)", length, ETH_MAX_PACKET_SIZE);
    ESP_RETURN_ON_ERROR(w5100_read_u16_stable(emac, W5100_REG_S0_TX_FSR, &free_size), TAG, "read TX FSR failed");
    if (length > free_size) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(w5100_read_u16(emac, W5100_REG_S0_TX_WR, &ptr), TAG, "read TX WR failed");
    ESP_RETURN_ON_ERROR(w5100_write_buf(emac, ptr, buf, length), TAG, "write frame failed");
    ESP_RETURN_ON_ERROR(w5100_write_u16(emac, W5100_REG_S0_TX_WR, ptr + length), TAG, "write TX WR failed");
    ESP_RETURN_ON_ERROR(w5100_command(emac, W5100_SCR_SEND), TAG, "issue SEND command failed");

    // Wait for SEND_OK before the next SEND, see the W5500 driver patch. The
    // W5100 doesn't tell the link speed: allow twice the 10 Mbit wire time
    // (preamble, padding, FCS, IFG) plus 1 ms.
    uint32_t wire_bytes = (length < 60 ? 60 : length) + 24;
    int64_t deadline = esp_timer_get_time() + wire_bytes * 8 / 10 * 2 + 1000;
    // A read can wait ~4 ms for the lock while the RX task copies a frame:
    // check the result before the deadline.
    for (;;) {
        uint8_t ir;
        ESP_RETURN_ON_ERROR(w5100_read(emac, W5100_REG_S0_IR, &ir, 1), TAG, "read S0_IR failed");
        if (ir & W5100_SIR_SEND_OK) {
            break;
        }
        if (esp_timer_get_time() > deadline) {
            return ESP_FAIL;
        }
    }
    return w5100_write_u8(emac, W5100_REG_S0_IR, W5100_SIR_SEND_OK);
}

static esp_err_t emac_w5100_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    return ESP_ERR_NOT_SUPPORTED;  // frames are pushed to the stack by emac_w5100_task
}

// Closing and reopening the socket empties the RX buffer.
static void w5100_restart_socket(emac_w5100_t *emac)
{
    w5100_command(emac, W5100_SCR_CLOSE);
    w5100_command(emac, W5100_SCR_OPEN);
}

// Reads one frame from the RX buffer and passes it to the stack. Returns
// false when the buffer is empty or unreadable.
static bool w5100_receive_frame(emac_w5100_t *emac)
{
    uint16_t received, ptr, len;
    uint8_t head[2];
    if (w5100_read_u16_stable(emac, W5100_REG_S0_RX_RSR, &received) != ESP_OK || received == 0) {
        return false;
    }
    if (w5100_read_u16(emac, W5100_REG_S0_RX_RD, &ptr) != ESP_OK || w5100_read_buf(emac, ptr, head, 2) != ESP_OK) {
        return false;
    }
    len = head[0] << 8 | head[1];  // includes the 2-byte header
    if (len < 2 + ETH_HEADER_LEN || len - 2 > ETH_MAX_PACKET_SIZE || len > received) {
        ESP_LOGE(TAG, "bad frame length %u (%u bytes received), dropping RX buffer", len, received);
        w5100_restart_socket(emac);
        return false;
    }
    uint8_t *frame = malloc(len - 2);
    if (!frame) {
        ESP_LOGE(TAG, "no mem for receive buffer");
    } else if (w5100_read_buf(emac, ptr + 2, frame, len - 2) != ESP_OK) {
        free(frame);
        return false;
    }
    if (w5100_write_u16(emac, W5100_REG_S0_RX_RD, ptr + len) != ESP_OK || w5100_command(emac, W5100_SCR_RECV) != ESP_OK) {
        free(frame);
        return false;
    }
    if (frame) {
        emac->eth->stack_input(emac->eth, frame, len - 2);
    }
    return true;
}

IRAM_ATTR static void w5100_isr_handler(void *arg)
{
    emac_w5100_t *emac = (emac_w5100_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void w5100_poll_timer(void *arg)
{
    emac_w5100_t *emac = (emac_w5100_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static void emac_w5100_task(void *arg)
{
    emac_w5100_t *emac = (emac_w5100_t *)arg;
    for (;;) {
        if (emac->int_gpio_num >= 0) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 && gpio_get_level(emac->int_gpio_num) != 0) {
                continue;
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        uint8_t ir = 0;
        if (w5100_read(emac, W5100_REG_S0_IR, &ir, 1) != ESP_OK || !(ir & W5100_SIR_RECV)) {
            continue;
        }
        // Clear before reading, so a frame arriving meanwhile sets it again
        w5100_write_u8(emac, W5100_REG_S0_IR, W5100_SIR_RECV);
        while (w5100_receive_frame(emac)) {
        }
    }
}

static esp_err_t emac_w5100_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    if (emac->int_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(emac->int_gpio_num);
        gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_NEGEDGE); // active low
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, w5100_isr_handler, emac);
    }
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    ESP_GOTO_ON_ERROR(w5100_reset(emac), err, TAG, "reset w5100 failed");
    ESP_GOTO_ON_ERROR(w5100_verify_id(emac), err, TAG, "verify chip ID failed");
    ESP_GOTO_ON_ERROR(w5100_setup_default(emac), err, TAG, "w5100 default setup failed");
    return ESP_OK;
err:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_w5100_deinit(esp_eth_mac_t *mac)
{
    emac_w5100_t *emac = __containerof(mac, emac_w5100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    if (emac->poll_timer && esp_timer_is_active(emac->poll_timer)) {
        esp_timer_stop(emac->poll_timer);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static void emac_w5100_free(emac_w5100_t *emac)
{
    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }
    if (emac->rx_task_hdl) {
        vTaskDelete(emac->rx_task_hdl);
    }
    if (emac->bus_acquired) {
        spi_device_release_bus(emac->spi);
    }
    if (emac->spi) {
        spi_bus_remove_device(emac->spi);
    }
    if (emac->lock) {
        vSemaphoreDelete(emac->lock);
    }
    free(emac);
}

static esp_err_t emac_w5100_del(esp_eth_mac_t *mac)
{
    emac_w5100_free(__containerof(mac, emac_w5100_t, parent));
    return ESP_OK;
}

// Takes the W5500's config struct: the W5100 needs the same settings.
// custom_spi_driver is not supported.
esp_eth_mac_t *bridge_eth_mac_new_w5100(const eth_w5500_config_t *config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_w5100_t *emac = NULL;
    ESP_GOTO_ON_FALSE(config && mac_config, NULL, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE((config->int_gpio_num >= 0) != (config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(emac_w5100_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "no mem for MAC instance");
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = config->int_gpio_num;
    emac->poll_period_ms = config->poll_period_ms;
    emac->parent.set_mediator = emac_w5100_set_mediator;
    emac->parent.init = emac_w5100_init;
    emac->parent.deinit = emac_w5100_deinit;
    emac->parent.start = emac_w5100_start;
    emac->parent.stop = emac_w5100_stop;
    emac->parent.del = emac_w5100_del;
    emac->parent.write_phy_reg = emac_w5100_write_phy_reg;
    emac->parent.read_phy_reg = emac_w5100_read_phy_reg;
    emac->parent.set_addr = emac_w5100_set_addr;
    emac->parent.get_addr = emac_w5100_get_addr;
    emac->parent.set_speed = emac_w5100_set_speed;
    emac->parent.set_duplex = emac_w5100_set_duplex;
    emac->parent.set_link = emac_w5100_set_link;
    emac->parent.set_promiscuous = emac_w5100_set_promiscuous;
    emac->parent.set_peer_pause_ability = emac_w5100_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = emac_w5100_enable_flow_ctrl;
    emac->parent.transmit = emac_w5100_transmit;
    emac->parent.receive = emac_w5100_receive;

    emac->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(emac->lock, NULL, err, TAG, "create lock failed");
    ESP_GOTO_ON_FALSE(w5100_spi_init(emac, config), NULL, err, TAG, "SPI initialization failed");

    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_w5100_task, "w5100_tsk", mac_config->rx_task_stack_size, emac,
                           mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create w5100 task failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = w5100_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }
    return &(emac->parent);

err:
    if (emac) {
        emac_w5100_free(emac);
    }
    return ret;
}

// ---------------------------------------------------------------- PHY

typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    uint32_t addr;
    int reset_gpio_num;
    eth_link_t link_status;
} phy_w5100_t;

static esp_err_t w5100_phy_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    ESP_RETURN_ON_FALSE(eth, ESP_ERR_INVALID_ARG, TAG, "mediator can't be null");
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    w5100->eth = eth;
    return ESP_OK;
}

static esp_err_t w5100_phy_set_link(esp_eth_phy_t *phy, eth_link_t link)
{
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    if (w5100->link_status != link) {
        w5100->link_status = link;
        ESP_RETURN_ON_ERROR(w5100->eth->on_state_changed(w5100->eth, ETH_STATE_LINK, (void *)link), TAG, "change link failed");
    }
    return ESP_OK;
}

// The link state can't be read: report it up, as 100 Mbit full duplex
static esp_err_t w5100_phy_get_link(esp_eth_phy_t *phy)
{
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    esp_eth_mediator_t *eth = w5100->eth;
    if (w5100->link_status != ETH_LINK_UP) {
        ESP_RETURN_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)ETH_SPEED_100M), TAG, "change speed failed");
        ESP_RETURN_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)ETH_DUPLEX_FULL), TAG, "change duplex failed");
        ESP_RETURN_ON_ERROR(w5100_phy_set_link(phy, ETH_LINK_UP), TAG, "change link failed");
    }
    return ESP_OK;
}

static esp_err_t w5100_phy_reset(esp_eth_phy_t *phy)
{
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    w5100->link_status = ETH_LINK_DOWN;
    return ESP_OK;
}

static esp_err_t w5100_phy_reset_hw(esp_eth_phy_t *phy)
{
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    if (w5100->reset_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(w5100->reset_gpio_num);
        gpio_set_direction(w5100->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(w5100->reset_gpio_num, 0);
        esp_rom_delay_us(10); // datasheet minimum is 2 us
        gpio_set_level(w5100->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(10)); // PLL lock time
    }
    return ESP_OK;
}

static esp_err_t w5100_phy_autonego_ctrl(esp_eth_phy_t *phy, eth_phy_autoneg_cmd_t cmd, bool *autonego_en_stat)
{
    if (cmd == ESP_ETH_PHY_AUTONEGO_DIS) {
        return ESP_ERR_NOT_SUPPORTED;  // set by the board's OPMODE pins, not by software
    }
    *autonego_en_stat = true;
    return ESP_OK;
}

static esp_err_t w5100_phy_ok(esp_eth_phy_t *phy)
{
    return ESP_OK;
}

static esp_err_t w5100_phy_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    return ESP_OK;
}

static esp_err_t w5100_phy_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    w5100->addr = addr;
    return ESP_OK;
}

static esp_err_t w5100_phy_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    ESP_RETURN_ON_FALSE(addr, ESP_ERR_INVALID_ARG, TAG, "addr can't be null");
    phy_w5100_t *w5100 = __containerof(phy, phy_w5100_t, parent);
    *addr = w5100->addr;
    return ESP_OK;
}

static esp_err_t w5100_phy_advertise_pause_ability(esp_eth_phy_t *phy, uint32_t ability)
{
    return ESP_OK;
}

static esp_err_t w5100_phy_loopback(esp_eth_phy_t *phy, bool enable)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t w5100_phy_set_speed(esp_eth_phy_t *phy, eth_speed_t speed)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t w5100_phy_set_duplex(esp_eth_phy_t *phy, eth_duplex_t duplex)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t w5100_phy_del(esp_eth_phy_t *phy)
{
    free(__containerof(phy, phy_w5100_t, parent));
    return ESP_OK;
}

esp_eth_phy_t *bridge_eth_phy_new_w5100(const eth_phy_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, NULL, TAG, "invalid arguments");
    phy_w5100_t *w5100 = calloc(1, sizeof(phy_w5100_t));
    ESP_RETURN_ON_FALSE(w5100, NULL, TAG, "no mem for PHY instance");
    w5100->addr = config->phy_addr;
    w5100->reset_gpio_num = config->reset_gpio_num;
    w5100->link_status = ETH_LINK_DOWN;
    w5100->parent.reset = w5100_phy_reset;
    w5100->parent.reset_hw = w5100_phy_reset_hw;
    w5100->parent.init = w5100_phy_ok;
    w5100->parent.deinit = w5100_phy_ok;
    w5100->parent.set_mediator = w5100_phy_set_mediator;
    w5100->parent.autonego_ctrl = w5100_phy_autonego_ctrl;
    w5100->parent.get_link = w5100_phy_get_link;
    w5100->parent.set_link = w5100_phy_set_link;
    w5100->parent.pwrctl = w5100_phy_pwrctl;
    w5100->parent.get_addr = w5100_phy_get_addr;
    w5100->parent.set_addr = w5100_phy_set_addr;
    w5100->parent.advertise_pause_ability = w5100_phy_advertise_pause_ability;
    w5100->parent.loopback = w5100_phy_loopback;
    w5100->parent.set_speed = w5100_phy_set_speed;
    w5100->parent.set_duplex = w5100_phy_set_duplex;
    w5100->parent.del = w5100_phy_del;
    return &(w5100->parent);
}

