// USB serial console for boards without a USB-Serial/JTAG peripheral
// (ESP32-S2), using TinyUSB CDC-ACM.
//
// ESP-IDF's built-in USB CDC console (the ROM driver) stopped sending under
// bridge traffic in testing, so it isn't used. TinyUSB doesn't handle
// esptool's reset requests by itself, so the DTR/RTS handling of the ROM
// console is copied here: RTS released while DTR is held reboots into the
// download mode, so `pio run -t upload` works without pressing buttons.

#include "sdkconfig.h"

#if CONFIG_TINYUSB_CDC_ENABLED

#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/usb_wrap_ll.h"
#include "soc/rtc_cntl_reg.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

// Runs in the TinyUSB task. The bridge has no state to save, so this resets
// immediately instead of running esp_restart()'s shutdown handlers (an early
// version that used them hung). The data lines are first pulled low long
// enough for the host to register a disconnect; otherwise it keeps the old
// enumeration and the ROM's USB stack can't take over in download mode.
// The override survives the system reset, so it's released just before.
static void reboot(bool download)
{
    const usb_wrap_pull_override_vals_t detached = {.dp_pd = true, .dm_pd = true};
    usb_wrap_ll_phy_enable_pull_override(&USB_WRAP, &detached);
    vTaskDelay(pdMS_TO_TICKS(200));
    if (download) REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    periph_module_reset(PERIPH_USB_MODULE);
    periph_module_enable(PERIPH_USB_MODULE);
    usb_wrap_ll_phy_disable_pull_override(&USB_WRAP);
    esp_rom_software_reset_system();
}

static void line_state_changed(int itf, cdcacm_event_t *event)
{
    static bool prev_dtr, prev_rts;
    bool dtr = event->line_state_changed_data.dtr;
    bool rts = event->line_state_changed_data.rts;
    // esptool: RTS (EN) released while DTR (IO0) is held -> download mode;
    // RTS pulsed with DTR low -> plain reset. Closing the port drops both at
    // once, which must not reboot.
    if (prev_rts && !rts) {
        if (dtr) reboot(true);
        else if (!prev_dtr) reboot(false);
    }
    prev_dtr = dtr;
    prev_rts = rts;
}

// Arduino-style "1200 bps touch", which PlatformIO sends before uploading.
static void line_coding_changed(int itf, cdcacm_event_t *event)
{
    if (event->line_coding_changed_data.p_line_coding->bit_rate == 1200) reboot(true);
}

static SemaphoreHandle_t s_lock;

// ESP_LOG output goes straight into the CDC TX FIFO instead of through stdio.
// Never blocks: if the host isn't reading, the text is dropped.
static int usb_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len > (int)sizeof(buf) - 1) len = sizeof(buf) - 1;
    if (len > 0 && xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)buf, len);
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
        xSemaphoreGive(s_lock);
    }
    return len;
}

void usb_console_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    // The ROM sets these after its own USB console was used (e.g. esptool's
    // reset out of download mode), so a later system reset would leave the
    // USB peripheral as it was. Same as Arduino-ESP32's TinyUSB setup.
    REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_IO_MUX_RESET_DISABLE | RTC_CNTL_USB_RESET_DISABLE);
    periph_module_reset(PERIPH_USB_MODULE);
    periph_module_enable(PERIPH_USB_MODULE);
    const tinyusb_config_t usb_cfg = {0};
    ESP_ERROR_CHECK(tinyusb_driver_install(&usb_cfg));
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_line_state_changed = line_state_changed,
        .callback_line_coding_changed = line_coding_changed,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    esp_log_set_vprintf(usb_vprintf);
    // Give the host time to enumerate, so startup logs aren't lost.
    vTaskDelay(pdMS_TO_TICKS(2000));
}

#else

void usb_console_init(void) {}

#endif
