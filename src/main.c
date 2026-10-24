// Transparent WiFi <-> Ethernet bridge for a single wired device.
//
// The ESP32 joins the WiFi network as a station using the MAC address of
// the device plugged into the Ethernet port ("MAC cloning"), then copies raw
// Ethernet frames in both directions. The ESP itself has no IP address; the
// wired device gets its address straight from the WiFi network's router.
//
// The device MAC is learned from the frames it sends and never stored: a
// bridge powered up without its device must not join WiFi as that device.
// WiFi is left again when the device goes quiet or the link goes down, and
// nothing is bridged while the port looks like it is plugged into a network.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_eth.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// Patched copy of the ESP-IDF W5500 MAC driver, see w5500/w5500_mac.c.
esp_eth_mac_t *bridge_eth_mac_new_w5500(const eth_w5500_config_t *w5500_config, const eth_mac_config_t *mac_config);
// W5100 MAC and PHY driver, see w5100/w5100.c.
esp_eth_mac_t *bridge_eth_mac_new_w5100(const eth_w5500_config_t *config, const eth_mac_config_t *mac_config);
esp_eth_phy_t *bridge_eth_phy_new_w5100(const eth_phy_config_t *config);
void usb_console_init(void);  // no-op unless the board uses TinyUSB, see usb_console.c

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#error "Copy src/wifi_credentials.example.h to src/wifi_credentials.h and fill in your network"
#endif

// Board settings (Ethernet chip, pins, SPI clock, link mode) come from
// Kconfig, set per board in boards/<board>.defaults.
#if CONFIG_BRIDGE_PIN_MOSI < 0 || CONFIG_BRIDGE_PIN_MISO < 0 || CONFIG_BRIDGE_PIN_SCLK < 0 || CONFIG_BRIDGE_PIN_CS < 0
#error "Ethernet SPI pins not set: build one of the environments in platformio.ini"
#endif
#if CONFIG_BRIDGE_ETH_W5100
#define ETH_CHIP "W5100"
#define eth_mac_new bridge_eth_mac_new_w5100
#define eth_phy_new bridge_eth_phy_new_w5100
#define ETH_SPI_DMA SPI_DMA_DISABLED  // the W5100 driver fills the SPI registers itself
#else
#define ETH_CHIP "W5500"
#define eth_mac_new bridge_eth_mac_new_w5500
#define eth_phy_new esp_eth_phy_new_w5500
#define ETH_SPI_DMA SPI_DMA_CH_AUTO
#endif
#define ETH_POLL_MS 1  // used when INT is not connected
#if CONFIG_BRIDGE_ETH_SPEED_100M
#define ETH_FORCED_SPEED ETH_SPEED_100M
#else
#define ETH_FORCED_SPEED ETH_SPEED_10M
#endif
#if CONFIG_BRIDGE_ETH_DUPLEX_HALF
#define ETH_FORCED_DUPLEX ETH_DUPLEX_HALF
#else
#define ETH_FORCED_DUPLEX ETH_DUPLEX_FULL
#endif

#define ETH2WIFI_QUEUE_LEN 32
// WiFi delivers TCP bursts much faster than 10 Mbit Ethernet drains them;
// each queued frame holds one WiFi RX buffer (see sdkconfig.defaults).
#define WIFI2ETH_QUEUE_LEN 64
#define MAC_SWITCH_IDLE_US (10 * 1000 * 1000)  // old MAC must be silent this long before we re-clone
#define DEVICE_IDLE_LEAVE_US (15 * 60 * 1000000LL)  // leave WiFi when the device is silent this long
#define LINK_DOWN_LEAVE_US (10 * 1000 * 1000)     // or the Ethernet link is down this long
// Watch the port this long before cloning a new MAC, so a network has time to
// show more sources: the first frame on a LAN came from its router.
#define CLONE_SETTLE_MS 3000
// LAN guard: this many distinct source MACs within LAN_WINDOW_US means the
// port is on a network, not on one device (a LAN showed ~10 in 3 s; one
// device with a few VMs behind it shows 1-3). Bridging stays off until the
// port has been quiet for LAN_HOLD_US.
#define LAN_SOURCES 4
#define LAN_WINDOW_US (10 * 1000 * 1000)
#define LAN_HOLD_US (60 * 1000 * 1000)
#define STATS_INTERVAL_MS 10000
// Roaming: on a multi-AP network the bridge was seen stuck on a far AP at
// -86 dBm (~70 KB/s). Below ROAM_RSSI, scan every ROAM_CHECK_STATS stats
// intervals and reconnect if another AP is at least ROAM_GAIN_DB stronger.
#define ROAM_RSSI (-75)
#define ROAM_GAIN_DB 10
#define ROAM_CHECK_STATS 6

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
static int64_t s_lan_seen_us;       // last time the LAN guard tripped, 0 = never
static int64_t s_eth_down_us;       // when the Ethernet link went down

static struct {
    uint32_t eth2wifi, wifi2eth, eth2wifi_qfull, eth2wifi_drop, wifi2eth_qfull, wifi2eth_txerr, echo, foreign_src;
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

// Records a source MAC for the LAN guard; returns true when LAN_SOURCES
// distinct sources were seen within LAN_WINDOW_US. Call with s_mac_lock held.
static bool lan_guard(const uint8_t *src, int64_t now)
{
    static struct {
        uint8_t mac[6];
        int64_t at_us;
    } seen[2 * LAN_SOURCES];
    int slot = 0, recent = 0;
    for (int i = 0; i < 2 * LAN_SOURCES; i++) {
        if (memcmp(seen[i].mac, src, 6) == 0) {
            slot = i;
            break;
        }
        if (seen[i].at_us < seen[slot].at_us) slot = i;
    }
    memcpy(seen[slot].mac, src, 6);
    seen[slot].at_us = now;
    for (int i = 0; i < 2 * LAN_SOURCES; i++) {
        if (seen[i].at_us && now - seen[i].at_us < LAN_WINDOW_US) recent++;
    }
    return recent >= LAN_SOURCES;
}

// Called from the Ethernet driver's RX task. We own `buffer` and must free it.
static esp_err_t eth_rx(esp_eth_handle_t eth, uint8_t *buffer, uint32_t len, void *priv)
{
    const uint8_t *src = buffer + 6;
    int64_t now = esp_timer_get_time();
    bool forward = false, notify = false;
    // Echoes carry WiFi-side source MACs: never clone or count them.
    bool echo = len >= 14 && is_echo(src, now);

    if (len >= 14 && !(src[0] & 0x01) && !echo) {
        portENTER_CRITICAL(&s_mac_lock);
        if (lan_guard(src, now)) {
            notify = !s_lan_seen_us || now - s_lan_seen_us > LAN_HOLD_US;
            s_lan_seen_us = now;
        }
        if (s_have_mac && memcmp(src, s_clone_mac, 6) == 0) {
            s_clone_last_seen_us = now;
            forward = true;
        } else if (!s_have_mac || now - s_clone_last_seen_us > MAC_SWITCH_IDLE_US) {
            memcpy(s_candidate_mac, src, 6);
            notify = true;
        }
        portEXIT_CRITICAL(&s_mac_lock);
    }
    if (notify) xTaskNotifyGive(s_ctrl_task);

    if (!forward) {
        if (echo) {
            s_stats.echo++;
        } else {
            s_stats.foreign_src++;
            log_foreign_src(buffer, len);
        }
    } else if (s_wifi_up) {
        frame_t f = {.buf = buffer, .len = len};
        if (xQueueSend(s_eth2wifi_q, &f, 0) == pdTRUE) return ESP_OK;
        s_stats.eth2wifi_qfull++;
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
        // ESP_ERR_NO_MEM: chip TX buffer full, worth retrying.
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
        if (CONFIG_BRIDGE_WIFI_MAX_TX_POWER) esp_wifi_set_max_tx_power(CONFIG_BRIDGE_WIFI_MAX_TX_POWER);
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
        if (s_restarting || !s_wifi_started) break;
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
        s_eth_down_us = esp_timer_get_time();
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

// Stops WiFi and forgets the device MAC; the next frame from a device starts over.
static void wifi_leave(void)
{
    portENTER_CRITICAL(&s_mac_lock);
    s_have_mac = false;
    portEXIT_CRITICAL(&s_mac_lock);
    if (!s_wifi_started) return;
    s_wifi_started = false;
    s_wifi_up = false;
    esp_wifi_stop();
}

static bool lan_detected(int64_t now)
{
    portENTER_CRITICAL(&s_mac_lock);
    bool lan = s_lan_seen_us && now - s_lan_seen_us < LAN_HOLD_US;
    portEXIT_CRITICAL(&s_mac_lock);
    return lan;
}

// Clones new device MACs reported by the Ethernet RX path, and leaves WiFi
// when the port turns out to be on a network or the device is gone.
static void ctrl_task(void *arg)
{
    bool lan_logged = false;
    for (;;) {
        bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0;
        int64_t now = esp_timer_get_time();

        if (lan_detected(now)) {
            if (!lan_logged) {
                ESP_LOGE(TAG, "%d+ devices on the Ethernet port: it is on a network, not bridging. "
                         "Connect a single device.", LAN_SOURCES);
                lan_logged = true;
            }
            wifi_leave();
            continue;
        }
        if (lan_logged) {
            ESP_LOGI(TAG, "Ethernet port quiet again, waiting for a device");
            lan_logged = false;
        }
        if (s_have_mac) {
            bool idle = now - s_clone_last_seen_us > DEVICE_IDLE_LEAVE_US;
            bool link_down = !s_eth_up && now - s_eth_down_us > LINK_DOWN_LEAVE_US;
            if (idle || link_down) {
                ESP_LOGW(TAG, "Wired device %s, leaving WiFi", idle ? "silent for 15 minutes" : "unplugged");
                wifi_leave();
            }
        }
        if (!notified) continue;

        vTaskDelay(pdMS_TO_TICKS(CLONE_SETTLE_MS));
        if (lan_detected(esp_timer_get_time())) continue;  // logged on the next pass
        uint8_t mac[6];
        portENTER_CRITICAL(&s_mac_lock);
        memcpy(mac, s_candidate_mac, 6);
        bool same = s_have_mac && memcmp(mac, s_clone_mac, 6) == 0;
        portEXIT_CRITICAL(&s_mac_lock);
        if (same) continue;

        ESP_LOGI(TAG, "Wired device " MACSTR " detected, cloning its MAC on WiFi", MAC2STR(mac));
        wifi_start_with_mac(mac);
    }
}

// Scans for our SSID while connected to `cur`; if a clearly stronger AP
// exists, disconnects so the reconnect (sorted by signal) picks it.
static void maybe_roam(const wifi_ap_record_t *cur)
{
    static wifi_ap_record_t recs[16];
    wifi_scan_config_t scan = {.ssid = (uint8_t *)BRIDGE_WIFI_SSID};
    uint16_t n = sizeof(recs) / sizeof(recs[0]);
    if (esp_wifi_scan_start(&scan, true) != ESP_OK) return;
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) return;
    const wifi_ap_record_t *best = NULL;
    for (int i = 0; i < n; i++) {
        if (!best || recs[i].rssi > best->rssi) best = &recs[i];
    }
    if (best && best->rssi >= cur->rssi + ROAM_GAIN_DB && memcmp(best->bssid, cur->bssid, 6) != 0) {
        ESP_LOGW(TAG, "Roaming: AP " MACSTR " at %d dBm, current %d dBm", MAC2STR(best->bssid), best->rssi, cur->rssi);
        esp_wifi_disconnect();
    }
}

static void stats_task(void *arg)
{
    for (int n = 1;; n++) {
        vTaskDelay(pdMS_TO_TICKS(STATS_INTERVAL_MS));
        wifi_ap_record_t ap = {0};
        if (s_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(TAG, "AP " MACSTR " ch %d rssi %d dBm phy %s%s%s", MAC2STR(ap.bssid), ap.primary, ap.rssi,
                     ap.phy_11b ? "b" : "", ap.phy_11g ? "g" : "", ap.phy_11n ? "n" : "");
            if (ap.rssi < ROAM_RSSI && n % ROAM_CHECK_STATS == 0) maybe_roam(&ap);
        }
        ESP_LOGI(TAG, "wifi=%s eth=%s | eth->wifi %lu (qfull %lu, txerr %lu) | wifi->eth %lu (qfull %lu, txerr %lu) | echo %lu other-src %lu",
                 s_wifi_up ? "up" : "down", s_eth_up ? "up" : "down",
                 s_stats.eth2wifi, s_stats.eth2wifi_qfull, s_stats.eth2wifi_drop, s_stats.wifi2eth, s_stats.wifi2eth_qfull,
                 s_stats.wifi2eth_txerr, s_stats.echo, s_stats.foreign_src);
        ESP_LOGI(TAG, "internal heap free %u (min %u) | eth tx: avg %lld us, max %lld us, >5ms %lu",
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL), s_stats.wifi2eth ? s_stats.tx_us_total / s_stats.wifi2eth : 0,
                 s_stats.tx_us_max, s_stats.tx_slow);
    }
}

// Returns false if the Ethernet chip doesn't answer (wiring, power), so the caller can
// retry instead of aborting. A boot loop would also take the USB console down.
static bool eth_init(void)
{
    static bool bus_ready;
    spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_BRIDGE_PIN_MOSI,
        .miso_io_num = CONFIG_BRIDGE_PIN_MISO,
        .sclk_io_num = CONFIG_BRIDGE_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    if (!bus_ready) {
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, ETH_SPI_DMA));
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        bus_ready = true;
    }

    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_speed_hz = CONFIG_BRIDGE_SPI_MHZ * 1000 * 1000,
        .queue_size = 20,
        .spics_io_num = CONFIG_BRIDGE_PIN_CS,
    };
    eth_w5500_config_t chip = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &dev);  // the W5100 driver takes the same
    chip.int_gpio_num = CONFIG_BRIDGE_PIN_INT;
    if (chip.int_gpio_num < 0) chip.poll_period_ms = ETH_POLL_MS;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_stack_size = 4096;
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = CONFIG_BRIDGE_PIN_RST;

    esp_eth_mac_t *mac = eth_mac_new(&chip, &mac_cfg);
    esp_eth_phy_t *phy = eth_phy_new(&phy_cfg);
    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&cfg, &s_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, ETH_CHIP " not responding (%s), check wiring", esp_err_to_name(err));
        mac->del(mac);
        phy->del(phy);
        return false;
    }

    uint8_t own_mac[6];
    esp_read_mac(own_mac, ESP_MAC_ETH);
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, own_mac));
    bool promiscuous = true;  // accept frames for the wired device's MAC, not just the chip's
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_PROMISCUOUS, &promiscuous));
#if !CONFIG_BRIDGE_ETH_AUTONEGOTIATE
    bool autonego = false;
    eth_speed_t speed = ETH_FORCED_SPEED;
    eth_duplex_t duplex = ETH_FORCED_DUPLEX;
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_AUTONEGO, &autonego));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_SPEED, &speed));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_DUPLEX_MODE, &duplex));
#endif
    ESP_ERROR_CHECK(esp_eth_update_input_path(s_eth, eth_rx, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event, NULL));
    ESP_ERROR_CHECK(esp_eth_start(s_eth));
    return true;
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
            // The default fast scan joins the first AP found with this SSID,
            // which on a multi-AP network can be a far one.
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
}

void app_main(void)
{
    usb_console_init();
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
    xTaskCreate(stats_task, "stats", 4096, NULL, 1, NULL);

    wifi_init();
    while (!eth_init()) vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Waiting for the wired device to send a frame");
}
