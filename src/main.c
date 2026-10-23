// Transparent WiFi <-> Ethernet bridge for a single wired device.
//
// The ESP32-S3 joins the WiFi network as a station using the MAC address of
// the device plugged into the W5500 port ("MAC cloning"), then copies raw
// Ethernet frames in both directions. The ESP itself has no IP address; the
// wired device gets its address straight from the WiFi network's router.
//
// The device MAC is learned from the first frame it sends and kept in NVS,
// so after a reboot WiFi can come up before the device says anything.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_eth.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

// Patched copy of the ESP-IDF W5500 MAC driver, see w5500/w5500_mac.c.
esp_eth_mac_t *bridge_eth_mac_new_w5500(const eth_w5500_config_t *w5500_config, const eth_mac_config_t *mac_config);

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#error "Copy src/wifi_credentials.example.h to src/wifi_credentials.h and fill in your network"
#endif

// W5500 wiring, from the seller's schematic. RST has a 10k pull-down, so the
// driver must actively release it.
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS 10
#define PIN_INT 14
#define PIN_RST 15
#define W5500_SPI_HZ (36 * 1000 * 1000)
// Ethernet link mode. The W5500 on this board can't hold a 100 Mbit link:
// forced 100 Mbit gets no link, autonegotiation picks 100 Mbit and drops it
// every few seconds. See
// https://github.com/jeroenvermeulen/esp32-s3-poe-w5500-n16r8-dev-board
// So the link is forced to 10 Mbit. Duplex must match the peer: a standard
// autonegotiating peer falls back to half duplex against a forced link, but
// some (e.g. Realtek USB 2.5G adapters) only do 10 Mbit full. A mismatch
// shows up as slow WiFi->Ethernet throughput.
// ETH_AUTONEGOTIATE=1 also works with peers that "downshift" after failed
// 100 Mbit attempts: it flaps for ~15 s, then settles at 10 Mbit with the
// right duplex.
#ifndef ETH_AUTONEGOTIATE
#define ETH_AUTONEGOTIATE 0
#endif
#ifndef ETH_FORCE_SPEED
#define ETH_FORCE_SPEED ETH_SPEED_10M
#endif
#ifndef ETH_FORCE_DUPLEX
#define ETH_FORCE_DUPLEX ETH_DUPLEX_FULL
#endif

#define ETH2WIFI_QUEUE_LEN 32
// WiFi delivers TCP bursts much faster than 10 Mbit Ethernet drains them;
// each queued frame holds one WiFi RX buffer (see sdkconfig.defaults).
#define WIFI2ETH_QUEUE_LEN 64
#define MAC_SWITCH_IDLE_US (10 * 1000 * 1000)  // old MAC must be silent this long before we re-clone
#define STATS_INTERVAL_MS 10000

static const char *TAG = "bridge";

typedef struct {
    void *buf;
    uint16_t len;
    void *eb;  // WiFi RX buffer handle; NULL for frames from Ethernet
} frame_t;

static esp_eth_handle_t s_eth;
static QueueHandle_t s_eth2wifi_q, s_wifi2eth_q;
static TaskHandle_t s_ctrl_task;
static volatile bool s_wifi_up, s_eth_up, s_wifi_started, s_restarting;

static portMUX_TYPE s_mac_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_clone_mac[6];      // MAC used on WiFi (= wired device MAC)
static bool s_have_mac;
static uint8_t s_candidate_mac[6];  // newly seen MAC waiting for ctrl task
static int64_t s_clone_last_seen_us;

static struct {
    uint32_t eth2wifi, wifi2eth, eth2wifi_drop, wifi2eth_qfull, wifi2eth_txerr, echo, foreign_src;
    uint32_t tx_slow;
    int64_t tx_us_total, tx_us_max;
} s_stats;

// The W5500 receives a copy of every frame it transmits. Remember the source
// MACs of recently transmitted frames so those echoes can be told apart from
// a real second device on the wire.
#define ECHO_RING 16
#define ECHO_WINDOW_US (200 * 1000)
static struct {
    uint8_t src[6];
    int64_t at_us;
} s_sent[ECHO_RING];
static int s_sent_next;

// ---------------------------------------------------------------- NVS

static bool load_mac(uint8_t mac[6])
{
    nvs_handle_t h;
    size_t len = 6;
    if (nvs_open("bridge", NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_blob(h, "mac", mac, &len) == ESP_OK && len == 6;
    nvs_close(h);
    return ok;
}

static void save_mac(const uint8_t mac[6])
{
    nvs_handle_t h;
    if (nvs_open("bridge", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "mac", mac, 6);
    nvs_commit(h);
    nvs_close(h);
}

// ---------------------------------------------------------------- Ethernet -> WiFi

// Logs each distinct non-cloned source MAC once (small ring), so a second
// device or a host with VMs/bridges behind the port shows up in the log.
static void log_foreign_src(const uint8_t *frame, uint32_t len)
{
    static uint8_t seen[8][6];
    static int next;
    if (len < 14) return;
    for (int i = 0; i < 8; i++) {
        if (memcmp(seen[i], frame + 6, 6) == 0) return;
    }
    memcpy(seen[next], frame + 6, 6);
    next = (next + 1) % 8;
    ESP_LOGW(TAG, "Frame from other source " MACSTR " (ethertype %02x%02x) not bridged",
             MAC2STR(frame + 6), frame[12], frame[13]);
}

static bool is_echo(const uint8_t *src, int64_t now)
{
    for (int i = 0; i < ECHO_RING; i++) {
        if (now - s_sent[i].at_us < ECHO_WINDOW_US && memcmp(s_sent[i].src, src, 6) == 0) return true;
    }
    return false;
}

// Called from the W5500 driver's RX task. We own `buffer` and must free it.
static esp_err_t eth_rx(esp_eth_handle_t eth, uint8_t *buffer, uint32_t len, void *priv)
{
    const uint8_t *src = buffer + 6;
    int64_t now = esp_timer_get_time();
    bool forward = false, new_device = false;

    if (len >= 14 && !(src[0] & 0x01)) {
        portENTER_CRITICAL(&s_mac_lock);
        if (s_have_mac && memcmp(src, s_clone_mac, 6) == 0) {
            s_clone_last_seen_us = now;
            forward = true;
        } else if (!s_have_mac || now - s_clone_last_seen_us > MAC_SWITCH_IDLE_US) {
            memcpy(s_candidate_mac, src, 6);
            new_device = true;
        }
        portEXIT_CRITICAL(&s_mac_lock);
    }
    if (new_device) xTaskNotifyGive(s_ctrl_task);

    if (!forward) {
        if (is_echo(src, now)) {
            s_stats.echo++;
        } else {
            s_stats.foreign_src++;
            log_foreign_src(buffer, len);
        }
    } else if (s_wifi_up) {
        frame_t f = {.buf = buffer, .len = len};
        if (xQueueSend(s_eth2wifi_q, &f, 0) == pdTRUE) return ESP_OK;
        s_stats.eth2wifi_drop++;
    }
    free(buffer);
    return ESP_OK;
}

static void eth2wifi_task(void *arg)
{
    frame_t f;
    for (;;) {
        xQueueReceive(s_eth2wifi_q, &f, portMAX_DELAY);
        esp_err_t err = ESP_FAIL;
        for (int tries = 0; s_wifi_up && tries < 5; tries++) {
            err = esp_wifi_internal_tx(WIFI_IF_STA, f.buf, f.len);
            if (err != ESP_ERR_NO_MEM) break;
            vTaskDelay(1);
        }
        if (err == ESP_OK) s_stats.eth2wifi++;
        else s_stats.eth2wifi_drop++;
        free(f.buf);
    }
}

// ---------------------------------------------------------------- WiFi -> Ethernet

// Called from the WiFi task; must not block. The RX buffer is handed to
// wifi2eth_task, which frees it after the SPI transfer.
static esp_err_t wifi_rx(void *buffer, uint16_t len, void *eb)
{
    // Drop frames we sent ourselves (broadcasts echoed back by the AP).
    bool own = len >= 14 && memcmp((uint8_t *)buffer + 6, s_clone_mac, 6) == 0;
    frame_t f = {.buf = buffer, .len = len, .eb = eb};
    if (own || !s_eth_up || xQueueSend(s_wifi2eth_q, &f, 0) != pdTRUE) {
        if (!own && s_eth_up) s_stats.wifi2eth_qfull++;
        esp_wifi_internal_free_rx_buffer(eb);
    }
    return ESP_OK;
}

static void wifi2eth_task(void *arg)
{
    frame_t f;
    for (;;) {
        xQueueReceive(s_wifi2eth_q, &f, portMAX_DELAY);
        if (f.len >= 14) {
            memcpy(s_sent[s_sent_next].src, (uint8_t *)f.buf + 6, 6);
            s_sent[s_sent_next].at_us = esp_timer_get_time();
            s_sent_next = (s_sent_next + 1) % ECHO_RING;
        }
        // ESP_ERR_NO_MEM: W5500 TX buffer full, worth retrying.
        esp_err_t err = ESP_ERR_NO_MEM;
        for (int tries = 0; err == ESP_ERR_NO_MEM && tries < 5; tries++) {
            if (tries) vTaskDelay(1);
            int64_t t0 = esp_timer_get_time();
            err = esp_eth_transmit(s_eth, f.buf, f.len);
            int64_t dt = esp_timer_get_time() - t0;
            s_stats.tx_us_total += dt;
            if (dt > s_stats.tx_us_max) s_stats.tx_us_max = dt;
            if (dt > 5000) s_stats.tx_slow++;
        }
        if (err == ESP_OK) s_stats.wifi2eth++;
        else s_stats.wifi2eth_txerr++;
        esp_wifi_internal_free_rx_buffer(f.eb);
    }
}

// ---------------------------------------------------------------- events

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        esp_wifi_internal_reg_rxcb(WIFI_IF_STA, wifi_rx);
        s_wifi_up = true;
        ESP_LOGI(TAG, "WiFi connected to \"%s\" as " MACSTR, BRIDGE_WIFI_SSID, MAC2STR(s_clone_mac));
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *d = data;
        s_wifi_up = false;
        if (s_restarting) break;
        ESP_LOGW(TAG, "WiFi disconnected (reason %d), reconnecting", d->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
        break;
    }
    }
}

static void eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_eth_up = true;
        ESP_LOGI(TAG, "Ethernet link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_eth_up = false;
        ESP_LOGW(TAG, "Ethernet link down");
        break;
    }
}

// ---------------------------------------------------------------- setup

// (Re)starts WiFi with the given MAC. esp_wifi_set_mac() only works while
// the station interface is stopped.
static void wifi_start_with_mac(const uint8_t mac[6])
{
    if (s_wifi_started) {
        s_restarting = true;
        s_wifi_up = false;
        esp_wifi_stop();
        s_restarting = false;
    }
    portENTER_CRITICAL(&s_mac_lock);
    memcpy(s_clone_mac, mac, 6);
    s_have_mac = true;
    s_clone_last_seen_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_mac_lock);

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, mac));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_wifi_started = true;
}

// Waits for the Ethernet RX path to report a new device MAC.
static void ctrl_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t mac[6];
        portENTER_CRITICAL(&s_mac_lock);
        memcpy(mac, s_candidate_mac, 6);
        bool same = s_have_mac && memcmp(mac, s_clone_mac, 6) == 0;
        portEXIT_CRITICAL(&s_mac_lock);
        if (same) continue;

        ESP_LOGI(TAG, "Wired device " MACSTR " detected, cloning its MAC on WiFi", MAC2STR(mac));
        save_mac(mac);
        wifi_start_with_mac(mac);
    }
}

static void stats_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATS_INTERVAL_MS));
        wifi_ap_record_t ap = {0};
        if (s_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(TAG, "AP " MACSTR " ch %d rssi %d dBm phy %s%s%s", MAC2STR(ap.bssid), ap.primary, ap.rssi,
                     ap.phy_11b ? "b" : "", ap.phy_11g ? "g" : "", ap.phy_11n ? "n" : "");
        }
        ESP_LOGI(TAG, "wifi=%s eth=%s | eth->wifi %lu (drop %lu) | wifi->eth %lu (qfull %lu, txerr %lu) | echo %lu other-src %lu",
                 s_wifi_up ? "up" : "down", s_eth_up ? "up" : "down",
                 s_stats.eth2wifi, s_stats.eth2wifi_drop, s_stats.wifi2eth, s_stats.wifi2eth_qfull,
                 s_stats.wifi2eth_txerr, s_stats.echo, s_stats.foreign_src);
        ESP_LOGI(TAG, "heap free %lu (min %lu) | eth tx: avg %lld us, max %lld us, >5ms %lu",
                 (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(), s_stats.wifi2eth ? s_stats.tx_us_total / s_stats.wifi2eth : 0,
                 s_stats.tx_us_max, s_stats.tx_slow);
    }
}

static void eth_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_speed_hz = W5500_SPI_HZ,
        .queue_size = 20,
        .spics_io_num = PIN_CS,
    };
    eth_w5500_config_t w5500 = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &dev);
    w5500.int_gpio_num = PIN_INT;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_stack_size = 4096;
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = PIN_RST;

    esp_eth_mac_t *mac = bridge_eth_mac_new_w5500(&w5500, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&cfg, &s_eth));

    uint8_t own_mac[6];
    esp_read_mac(own_mac, ESP_MAC_ETH);
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, own_mac));
    bool promiscuous = true;  // accept frames for the wired device's MAC, not just the W5500's
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_PROMISCUOUS, &promiscuous));
#if !ETH_AUTONEGOTIATE
    bool autonego = false;
    eth_speed_t speed = ETH_FORCE_SPEED;
    eth_duplex_t duplex = ETH_FORCE_DUPLEX;
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_AUTONEGO, &autonego));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_SPEED, &speed));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_DUPLEX_MODE, &duplex));
#endif
    ESP_ERROR_CHECK(esp_eth_update_input_path(s_eth, eth_rx, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event, NULL));
    ESP_ERROR_CHECK(esp_eth_start(s_eth));
}

static void wifi_init(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t cfg = {
        .sta = {
            .ssid = BRIDGE_WIFI_SSID,
            .password = BRIDGE_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_eth2wifi_q = xQueueCreate(ETH2WIFI_QUEUE_LEN, sizeof(frame_t));
    s_wifi2eth_q = xQueueCreate(WIFI2ETH_QUEUE_LEN, sizeof(frame_t));
    xTaskCreate(ctrl_task, "ctrl", 4096, NULL, 5, &s_ctrl_task);
    xTaskCreate(eth2wifi_task, "eth2wifi", 4096, NULL, 15, NULL);
    xTaskCreate(wifi2eth_task, "wifi2eth", 4096, NULL, 15, NULL);
    xTaskCreate(stats_task, "stats", 3072, NULL, 1, NULL);

    wifi_init();
    eth_init();

    uint8_t mac[6];
    if (load_mac(mac)) {
        ESP_LOGI(TAG, "Using stored device MAC " MACSTR, MAC2STR(mac));
        wifi_start_with_mac(mac);
    } else {
        ESP_LOGI(TAG, "No device MAC stored yet; waiting for the wired device to send a frame");
    }
}
