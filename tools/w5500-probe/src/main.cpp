// W5500 pin finder: bit-bangs SPI on candidate GPIOs and looks for the
// W5500 VERSIONR register (0x0039), which always reads 0x04. Hits are
// confirmed by a write/readback of the gateway register (GAR0).

#include <Arduino.h>

struct Layout {
  int mosi, miso, sclk, cs, irq, rst;
  const char *name;
};

// Pinouts used by known ESP32-S3 + W5500 boards/examples.
static const Layout kKnown[] = {
    // This board, per seller schematic; RST has a 10k pull-down so it must be driven high.
    {11, 13, 12, 10, 14, 15, "AliExpress ESP32-S3 W5500 (schematic)"},
    {11, 12, 13, 14, 10, 9, "Waveshare ESP32-S3-ETH"},
    {11, 13, 12, 10, 14, 9, "Espressif / WIZnet example"},
    {12, 11, 10, 9, 13, 14, "LilyGO T-ETH-Lite S3"},
    {43, 44, 42, 41, 2, 1, "KinCony"},
};

// GPIOs scanned in brute-force mode. Skips strapping pins (0, 3, 45, 46),
// USB (19, 20) and flash/PSRAM (26-37).
static const int kPool[] = {1,  2,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
                            16, 17, 18, 21, 38, 39, 40, 41, 42, 43, 44, 47, 48};
static const int kPoolSize = sizeof(kPool) / sizeof(kPool[0]);

static int pMosi, pMiso, pSclk, pCs;
static String result = "no W5500 found";

static uint8_t xfer(uint8_t out) {  // SPI mode 0, MSB first
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(pMosi, (out >> i) & 1);
    digitalWrite(pSclk, HIGH);
    in = (in << 1) | digitalRead(pMiso);
    digitalWrite(pSclk, LOW);
  }
  return in;
}

static uint8_t readReg(uint16_t addr) {
  digitalWrite(pCs, LOW);
  xfer(addr >> 8);
  xfer(addr & 0xff);
  xfer(0x00);  // common block, read, variable length
  uint8_t v = xfer(0x00);
  digitalWrite(pCs, HIGH);
  return v;
}

static void writeReg(uint16_t addr, uint8_t v) {
  digitalWrite(pCs, LOW);
  xfer(addr >> 8);
  xfer(addr & 0xff);
  xfer(0x04);  // common block, write, variable length
  xfer(v);
  digitalWrite(pCs, HIGH);
}

static void attach(int mosi, int miso, int sclk, int cs) {
  pMosi = mosi; pMiso = miso; pSclk = sclk; pCs = cs;
  pinMode(pCs, OUTPUT);
  digitalWrite(pCs, HIGH);
  pinMode(pSclk, OUTPUT);
  digitalWrite(pSclk, LOW);
  pinMode(pMosi, OUTPUT);
  pinMode(pMiso, INPUT);
}

static void detach() {
  pinMode(pMosi, INPUT);
  pinMode(pSclk, INPUT);
  pinMode(pCs, INPUT);
}

static bool looksLikeW5500() {
  if (readReg(0x0039) != 0x04) return false;
  if (readReg(0x0039) != 0x04) return false;
  uint8_t old = readReg(0x0001);
  writeReg(0x0001, 0xA5);
  bool ok = readReg(0x0001) == 0xA5;
  writeReg(0x0001, 0x5A);
  ok = ok && readReg(0x0001) == 0x5A;
  writeReg(0x0001, old);
  return ok;
}

static bool tryKnown() {
  for (const Layout &l : kKnown) {
    Serial.printf("Trying %s ...\n", l.name);
    pinMode(l.rst, OUTPUT);  // pulse reset in case RST is GPIO-driven
    digitalWrite(l.rst, LOW);
    delay(2);
    digitalWrite(l.rst, HIGH);
    delay(100);
    attach(l.mosi, l.miso, l.sclk, l.cs);
    bool hit = looksLikeW5500();
    detach();
    if (hit) {
      result = String("W5500 FOUND (") + l.name + "): MOSI=" + l.mosi + " MISO=" + l.miso +
               " SCLK=" + l.sclk + " CS=" + l.cs + " (INT/RST likely " + l.irq + "/" + l.rst + ")";
      return true;
    }
    pinMode(l.rst, INPUT);
  }
  return false;
}

static bool bruteForce() {
  Serial.println("Known layouts failed, scanning GPIO combinations...");
  for (int c = 0; c < kPoolSize; c++) {
    Serial.printf("  CS candidate GPIO%d\n", kPool[c]);
    for (int s = 0; s < kPoolSize; s++) {
      if (s == c) continue;
      for (int o = 0; o < kPoolSize; o++) {
        if (o == c || o == s) continue;
        for (int i = 0; i < kPoolSize; i++) {
          if (i == c || i == s || i == o) continue;
          attach(kPool[o], kPool[i], kPool[s], kPool[c]);
          bool hit = looksLikeW5500();
          detach();
          if (hit) {
            result = String("W5500 FOUND (scan): MOSI=") + kPool[o] + " MISO=" + kPool[i] +
                     " SCLK=" + kPool[s] + " CS=" + kPool[c];
            return true;
          }
        }
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(3000);  // give the host time to open the USB serial port
  Serial.println("\n=== W5500 pin finder ===");
  if (!tryKnown()) bruteForce();
  Serial.println(result);
}

void loop() {
  delay(3000);
  Serial.println(result);
}
