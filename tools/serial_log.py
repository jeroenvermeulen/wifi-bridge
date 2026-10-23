"""Print the board's USB serial log without resetting it.

The ESP32-S3 USB-Serial/JTAG resets the chip when RTS is asserted while DTR
is not. macOS asserts both on open (harmless), and pyserial would then drop
DTR before RTS, briefly hitting the reset state. So open with both asserted
and release RTS first. Waits for the port to (re)appear, since it
re-enumerates on every chip reset.

Usage: python tools/serial_log.py [PORT] [SECONDS] [STOP_TEXT]
"""
import glob
import sys
import time

import serial

want = sys.argv[1] if len(sys.argv) > 1 else ""
secs = float(sys.argv[2]) if len(sys.argv) > 2 else float("inf")
stop = sys.argv[3] if len(sys.argv) > 3 else None

end = time.time() + secs
s = None
while time.time() < end:
    try:
        if s is None:
            port = want or (glob.glob("/dev/cu.usbmodem*") or [""])[0]
            if not port:
                time.sleep(0.3)
                continue
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = port, 115200, 1
            s.open()
            s.rts = False
            s.dtr = False
        line = s.readline().decode(errors="replace").rstrip()
        if line:
            print(line, flush=True)
            if stop and stop in line:
                break
    except (serial.SerialException, OSError):
        s = None
        time.sleep(0.3)
