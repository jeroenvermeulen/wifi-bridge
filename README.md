# wifi-bridge

Firmware that turns a cheap ESP32-S3 + W5500 board into a transparent WiFi-to-Ethernet bridge
for one wired device. The device behaves as if it were on the WiFi network itself: it gets its
IP address from the WiFi network's router, and other hosts can reach it directly.

Board: the AliExpress "ESP32-S3 PoE W5500 N16R8" development board. See
[esp32-s3-poe-w5500-n16r8-dev-board](https://github.com/jeroenvermeulen/esp32-s3-poe-w5500-n16r8-dev-board)
for its hardware documentation.

## How it works

- The bridge learns the wired device's MAC address from the first frame the device sends, and
  stores it in flash.
- It joins the WiFi network as a station *using that MAC address* (MAC cloning). WiFi client
  mode can only carry one MAC address, which is why this supports a single device.
- Raw Ethernet frames are copied unchanged between the W5500 and WiFi. The ESP32 has no IP
  address of its own.
- If a different device is plugged in, the bridge switches to its MAC once the old device has
  been silent for 10 seconds.

## Performance

Measured through a Realtek USB 2.5G adapter on a Mac, to the internet over WiFi (-61 dBm):

| | |
|---|---|
| Download (WiFi → Ethernet) | ~700 KB/s |
| Upload (Ethernet → WiFi) | ~1.0 MB/s |
| Ping to the router | ~7 ms average, 0% loss |

The limit is the Ethernet link: the W5500 on this board can't hold 100 Mbit, so the link is
forced to 10 Mbit.

## Setup

1. Install [PlatformIO](https://platformio.org/). ESP-IDF is downloaded on the first build.
2. Copy `src/wifi_credentials.example.h` to `src/wifi_credentials.h` and fill in your network.
   That file is gitignored.
3. Connect the board's **ESP-USB** port with a **USB-A to USB-C** cable (USB-C to USB-C gives no
   power on this board), switch it on, and flash:
   ```
   pio run -t upload
   ```
4. Plug the device into the RJ45 port. **Never connect the RJ45 port to the network the bridge's
   WiFi joins**: that creates a loop.

## Configuration

Settings are compile-time `#define`s in [src/main.c](src/main.c). Most can be overridden without
editing, for example `PLATFORMIO_BUILD_FLAGS="-DETH_AUTONEGOTIATE=1" pio run -t upload`.

| Setting | Default | Notes |
|---|---|---|
| `ETH_FORCE_DUPLEX` | `ETH_DUPLEX_FULL` | Must match the wired device. Use `ETH_DUPLEX_HALF` if WiFi → Ethernet is slow and the device autonegotiates normally. |
| `ETH_AUTONEGOTIATE` | `0` | `1` works with devices that downshift to 10 Mbit after failed 100 Mbit attempts (link flaps for ~15 s first). |
| `ETH_FORCE_SPEED` | `ETH_SPEED_10M` | 100 Mbit gets no link on this board. |

To forget the learned device MAC, erase the NVS partition:
`esptool --port /dev/cu.usbmodem* erase-region 0x9000 0x6000`.

## Monitoring

The bridge logs over USB serial every 10 seconds: link states, signal strength, frame counters
per direction, drops, and free memory.

```
~/.platformio/penv/bin/python tools/serial_log.py
```

This opens the port without resetting the board, unlike most serial monitors.
[tools/bridge_test.sh](tools/bridge_test.sh) runs a ping, download and upload test bound to the Mac
interface that is wired to the bridge (default `en20`).

## W5500 driver patch

[src/w5500/w5500_mac.c](src/w5500/w5500_mac.c) is a patched copy of the ESP-IDF 5.4.1 W5500 MAC
driver. Upstream stops waiting for "send complete" after about 10 SPI reads, which is before a
full-size frame has left a 10 Mbit wire. The next frame is then started mid-transmission and
lost, which cost about 35% of downloaded frames. The patched driver waits for as long as the frame
needs on the wire. It also polls the command register instead of sleeping 10 ms.

## Tools

- [tools/w5500-probe](tools/w5500-probe): Arduino sketch that finds which GPIOs a W5500 is wired
  to, by bit-banging SPI and reading the chip's version register.
- [tools/serial_log.py](tools/serial_log.py): serial log reader that doesn't reset the board.
- [tools/bridge_test.sh](tools/bridge_test.sh): traffic test through the bridge.
