# AGENTS.md

Notes for AI agents working on this repo. Read this first; it should save you
from re-exploring the codebase and from re-learning the hardware the hard way.

## What this is

ESP-IDF firmware (built with PlatformIO) that bridges one wired device to a
WiFi network. The ESP32 joins WiFi as a station with the wired device's MAC
address (MAC cloning) and copies raw Ethernet frames between a W5500 (SPI
Ethernet) and WiFi. The ESP has no IP address of its own. See
[README.md](README.md) for the user-facing description.

## Layout

| Path | Purpose |
|------|---------|
| `src/main.c` | Everything bridge-related: MAC learning (NVS), RX callbacks, the two forwarding tasks, WiFi/Ethernet setup, stats log every 10 s |
| `src/w5500/w5500_mac.c` | Patched copy of the ESP-IDF 5.4.1 W5500 MAC driver (changes marked `wifi-bridge:`) |
| `src/usb_console.c` | TinyUSB CDC console for boards without USB-Serial/JTAG (ESP32-S2); no-op elsewhere |
| `src/Kconfig.projbuild` | The bridge's board options (`CONFIG_BRIDGE_*`) |
| `src/idf_component.yml` | Pulls `esp_tinyusb`, for the esp32s2 target only |
| `boards/<env>.defaults` | Per-board sdkconfig fragment: pins, SPI clock, link mode, TX power, chip options |
| `sdkconfig.defaults` | Settings shared by all boards (WiFi buffer counts etc.) |
| `platformio.ini` | One environment per board; each points `board_build.cmake_extra_args` at its defaults file |
| `src/wifi_credentials.h` | Gitignored; copy from `wifi_credentials.example.h`. Never commit real credentials |
| `tools/` | `serial_log.py` (log reader that doesn't reset the board), `bridge_test.sh` (traffic test), `w5500-probe` (finds W5500 GPIOs) |

Generated, all gitignored: `sdkconfig.<env>`, `managed_components/`,
`dependencies.lock`, `.pio/`.

## Boards

| Env | Hardware | Docs |
|-----|----------|------|
| `esp32-s3-poe` (default) | ESP32-S3 PoE W5500 N16R8 board | https://github.com/jeroenvermeulen/esp32-s3-poe-w5500-n16r8-dev-board |
| `lolin-s2-mini` | LOLIN S2 Mini (ESP32-S2FN4R2) + USR-ES1 W5500 module on wires | https://github.com/jeroenvermeulen/esp32-w5500 |

**Board differences belong in `boards/*.defaults`, never in `#if` blocks in
the code.** The user explicitly asked for this. If a board needs a new knob,
add a `CONFIG_BRIDGE_*` option to `Kconfig.projbuild` (with a neutral
default) and set it in the board file. Feature checks on chip options (e.g.
`CONFIG_TINYUSB_CDC_ENABLED`) are fine.

ESP-IDF applies defaults only to a fresh config: after editing a defaults
file, delete `sdkconfig.<env>` before building, or the change is silently
ignored.

## Build and verify

```sh
pio run -e esp32-s3-poe
pio run -e lolin-s2-mini
```

Build both environments after any change to shared code. The first configure
after switching envs can fail once (the shared `dependencies.lock` changes
target); just run it again.

## Testing on hardware

The user lets agents flash and test. Typical setup on the user's Mac:

- Board USB at `/dev/cu.usbmodem*`. S2 app port: `/dev/cu.usbmodem1234561`
  (TinyUSB). S2 ROM download mode: `/dev/cu.usbmodem01`.
- The bridge's RJ45 port is wired to the Mac's interface `en20`.
- Flash: `pio run -e <env> -t upload`.
- Logs: `~/.platformio/penv/bin/python tools/serial_log.py [PORT] [SECONDS]`.
  Always pass SECONDS, and check `lsof /dev/cu.usbmodem*` first: two readers
  on one port each get about every other byte ("internal" arrives as "nenl"),
  which looks like a firmware console bug but isn't.
- The user's WiFi has several APs. When performance is bad, check the "AP ...
  rssi" stats line first: -86 dBm on a far AP looked like a firmware
  regression (~70 KB/s, uploads stalled).
  `pio device monitor` and plain serial tools may reset the board.
- Traffic test: `sh tools/bridge_test.sh en20` (ping, 5 MB down, 3 MB up).
- Link state: `ifconfig en20 | grep media`.

Wrap long-running commands in `timeout`; a hung esptool otherwise blocks for
minutes.

## Hardware quirks (learned the hard way)

ESP32-S3 PoE board:
- Its W5500 can't hold 100 Mbit, so the link is forced to 10 Mbit full.
- Needs a USB-A to USB-C cable; RST has a pull-down that the driver releases.

LOLIN S2 Mini:
- **WiFi TX power must stay capped** (`CONFIG_BRIDGE_WIFI_MAX_TX_POWER=34`,
  8.5 dBm). At full power, uploads drop to ~75 KB/s and the USB console dies
  under traffic. That is probably supply dips on the board's small regulator,
  which also feeds the W5500.
- Single core, ~95 KB free internal RAM: WiFi/LWIP buffers are in PSRAM.
  Watch the "internal heap free (min ...)" stats line when changing buffer
  counts or queue lengths.
- Console is TinyUSB, logging via `esp_log_set_vprintf` straight into the CDC
  FIFO (newlib stdio over TinyUSB stalled). The TinyUSB task runs at priority
  20, above the bridge tasks (15); at its default priority it starves and the
  USB stack wedges.
- `reboot()` in `usb_console.c` handles PlatformIO's 1200 bps touch and
  esptool's DTR/RTS reset. Each line in it fixes a real failure (USB_WRAP
  pull override for a visible detach, clearing the ROM's
  `RTC_CNTL_USB_CONF` reset-disable bits at init, an immediate ROM reset).
  Don't simplify it without re-running the stress test below.
- **If the S2's USB stops responding, only the user can recover it** (hold
  0, press RST, release 0, then flash with
  `esptool --chip esp32s2 -p /dev/cu.usbmodem01 --before no-reset --after watchdog-reset write-flash ...`).
  Each failed experiment costs the user a button press, so batch changes and
  verify carefully before flashing anything that touches USB or reset.
- Stress test for USB/reset changes: 5× `pio run -e lolin-s2-mini -t upload`
  while running traffic through `en20`, once with a log reader open during
  the traffic.

## Conventions

- C, 4-space indent, `snake_case`. File-local functions and globals are
  `static`; static globals are prefixed `s_`. Short comments that explain
  *why*, often with the measurement that motivated them.
- Hot paths (RX callbacks) must not block: they queue or drop and count it in
  `s_stats`. Add a counter to the stats log for any new drop path.
- Keep the patched W5500 driver's `wifi-bridge:` markers so it can be
  re-diffed against upstream.
- Report results honestly with numbers from `bridge_test.sh` and the stats
  log; the README performance table comes from those.

## Keep in sync when you change behaviour

- `README.md`: board table, performance table, configuration table, S2 notes.
- This file: quirks and workflow, when you learn something new.
- README states the code is AI generated and tested by humans; keep that
  disclaimer.
