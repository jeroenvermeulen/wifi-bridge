# AGENTS.md

Notes for AI agents working on this repo. Read this first; it should save you
from re-exploring the codebase and from re-learning the hardware the hard way.

## What this is

ESP-IDF firmware (built with PlatformIO) that bridges one wired device to a
WiFi network. The ESP32 joins WiFi as a station with the wired device's MAC
address (MAC cloning) and copies raw Ethernet frames between a W5500 or W5100
(SPI Ethernet) and WiFi. The ESP has no IP address of its own. See
[README.md](README.md) for the user-facing description.

## Layout

| Path | Purpose |
|------|---------|
| `src/main.c` | Everything bridge-related: MAC learning (never stored), LAN guard, leaving WiFi when the device is gone, RX callbacks, the two forwarding tasks (eth->wifi drops frames with bad IPv4/TCP/UDP checksums), WiFi/Ethernet setup, stats log every 10 s |
| `src/w5500/w5500_mac.c` | Patched copy of the ESP-IDF 5.4.1 W5500 MAC driver (changes marked `wifi-bridge:`) |
| `src/w5100/w5100.c` | Our own W5100 MAC + PHY driver (ESP-IDF has none); drives the SPI registers directly |
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
| `esp32-uno-shield` | ESP32-WROOM-32U + Arduino Uno Ethernet Shield (W5100) on wires | README "ESP32 + Arduino Ethernet Shield notes" |
| `esp32-w5500` | ESP32-WROOM-32U + USR-ES1 W5500 module on wires (the user's permanent bridge) | README "ESP32 + W5500 module notes" |

**Board differences belong in `boards/*.defaults`, never in `#if` blocks in
the code.** The user explicitly asked for this. If a board needs a new knob,
add a `CONFIG_BRIDGE_*` option to `Kconfig.projbuild` (with a neutral
default) and set it in the board file. Feature checks on chip options (e.g.
`CONFIG_TINYUSB_CDC_ENABLED`, or `CONFIG_BRIDGE_ETH_W5100` for the Ethernet
chip, which main.c checks once to pick the driver) are fine.

ESP-IDF applies defaults only to a fresh config: after editing a defaults
file, delete `sdkconfig.<env>` before building, or the change is silently
ignored.

## Build and verify

```sh
pio run -e esp32-s3-poe
pio run -e lolin-s2-mini
pio run -e esp32-uno-shield
pio run -e esp32-w5500
```

Build all environments after any change to shared code. The first configure
after switching envs can fail once (the shared `dependencies.lock` changes
target); just run it again.

## Testing on hardware

The user lets agents flash and test. Typical setup on the user's Mac:

- Board USB at `/dev/cu.usbmodem*`. S2 app port: `/dev/cu.usbmodem1234561`
  (TinyUSB). S2 ROM download mode: `/dev/cu.usbmodem01`. ESP32 (Uno shield, W5500):
  `/dev/cu.usbserial-0001` (USB-serial chip with DTR/RTS auto-reset). Opening
  it resets the board, whatever pyserial does with DTR/RTS, so start
  `serial_log.py` first and wait for "WiFi connected" before a traffic test.
- The bridge's RJ45 port is wired to the Mac's interface `en20`.
  **Check `ifconfig en20 | grep status` says `active` before flashing bridge
  firmware.** The bridge clones the first source MAC it hears. On 2026-10-24
  the Uno shield's RJ45 turned out to be on the house LAN: it cloned the
  router's MAC, joined WiFi with it and bridged LAN traffic for ~2 minutes
  before it was erased. If the link isn't on `en20`, stop and ask the user.
  (The LAN guard in main.c now blocks that case, but don't rely on it.)
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
  It is too short to catch faults that happen once per 10-20 MB: on
  esp32-w5500 several "clean" runs turned out to be luck. For a verdict,
  run 16 uploads of 5 MB (`head -c 5000000 /dev/urandom | curl --interface
  en20 --data-binary @- https://speed.cloudflare.com/__up`) and 16
  downloads, then count curl failures (also TLS "bad record mac") and the
  W5500 driver's warnings in the log.
- Link state: `ifconfig en20 | grep media`.

Wrap long-running commands in `timeout`; a hung esptool otherwise blocks for
minutes.

## Hardware quirks (learned the hard way)

ESP32-S3 PoE board:
- Its W5500 can't hold 100 Mbit. It autonegotiates: the link flaps for ~15 s
  until the Mac's adapter downshifts to 10 Mbit. Forced 10 Mbit full lost
  ~86% of frames toward the Mac under two-way traffic (8 parallel uploads:
  2 reset); autoneg: none lost, ~1.1 MB/s up, ~720 KB/s down (5 MB, -63 dBm,
  2026-10-24).
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

ESP32 + Arduino Uno Ethernet Shield (W5100):
- ESP-IDF's default MISO timing (`input_delay_ns` 0) makes it sample half a
  clock late, which on this shield shifted every byte read right by one bit
  (RMSR reset value 0x55 read as 0x2a). The driver sets 40 ns. A 64 KB
  write/read-back test passed at 25-70 ns and failed at 0-20 ns.
- 13.3 MHz (rated 14) is used. 16 MHz read back clean, 20 MHz corrupted 7%
  of bytes.
- 3.5 us per byte (2.4 us of it is the 32-bit frame) with direct register
  access, 4.0 us through the HAL buffer helpers. `-O2` on the driver gained
  nothing more. The SPI bus must be non-DMA (main.c picks that for W5100).
- No link status register: `eth=up` always. MACRAW receives every frame on
  the wire (no MAC filter), but not its own transmissions (`echo 0`).
- Measured 2026-10-24 through en20 at -43 dBm: ~220 KB/s down, 130-150 KB/s
  up, ping 12 ms, no queue drops or TX errors.

ESP32 + W5500 module (esp32-w5500), first a blue AliExpress module, now a USR-ES1:
- Blue module: the INT wire's edges reset the W5500 (3-8 times per 80 MB of
  uploads), also with the firmware polling and ignoring INT. Keeping the
  chip's INT output quiet (the driver does that when
  `CONFIG_BRIDGE_PIN_INT=-1`) stopped it: 0 resets in 160 MB. Ruled out
  first: SPI clock (20/26/40 MHz all failed), GPIO drive strength, WiFi TX
  power (5 and 13 dBm both failed). The USR-ES1 runs with INT: 0 resets.
- Forced 10 Mbit full: ~80% of frames from the bridge to the Mac's adapter
  were lost under two-way traffic (compare `netstat -I en20` Ipkts with the
  bridge's wifi->eth count), so speedtest.net's parallel uploads stalled.
  Single streams hid it. Forced half duplex: the W5500 closed its socket in
  a loop (218 reopens) and the bridge stopped forwarding. The S3 board had
  the same loss (~86%) at forced 10 Mbit full; it now autonegotiates (see
  below). Likely the adapter falls back to half duplex against a forced link
  while macOS keeps showing full-duplex: losses are only toward the Mac and
  only while it transmits.
- At 100 Mbit about 1 in 60000 frames from the Mac leaves the W5500 with a
  bad TCP checksum (both modules). Located by checking checksums in the
  bridge plus a task that read a known pattern from the W5500's socket 1
  TX buffer during the same traffic: ~500000 reads, no errors except one
  bit at WiFi startup. Long static read tests (600 MB, any length) were
  clean. Some bad frames reached Cloudflare unchecked and failed uploads
  with a TLS "bad record mac". The checksum check in eth2wifi_task fixes
  that: 48 uploads of 5 MB, no failures, `badsum` counts the drops.
- The W5500 driver's transmit wait used to fail frames that completed after
  the deadline (bridge task preempted by WiFi). That left SEND_OK set and
  closed the socket every few seconds during 10 Mbit downloads. Fixed for
  all W5500 boards; the S3 board wasn't retested.
- Measured 2026-10-24 with the USR-ES1 through en20 at -37 to -44 dBm:
  ~1.4 MB/s down, ~820 KB/s up (5 MB; ~570 KB/s in bridge_test's 3 MB),
  ping ~8 ms, 8 parallel uploads ~0.9 MB/s shared evenly with 0% ping loss.

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
