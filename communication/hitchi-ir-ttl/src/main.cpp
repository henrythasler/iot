#include <Arduino.h>
#include <WiFi.h>

constexpr int ledBuiltin = LED_BUILTIN;

constexpr int RX1 = 4; // GPIO 4 (Header J3, Pin 10)
constexpr int TX1 = 5; // GPIO 5 (Header J3, Pin 11)

uint32_t initStage = 0;

// Task Scheduler
uint32_t timestamp = 0;
uint32_t task1sTimer = 0;
uint32_t counter1s = 0;

uint32_t task50msTimer = 0;
uint32_t counter50ms = 0;

// multi-purpose static bytearray
uint8_t txBuffer[256];
uint8_t rxBuffer[256];

float rgbCol[3];

// HSV->RGB conversion based on GLSL version
// from https://gist.github.com/postspectacular/2a4a8db092011c6743a7
// expects hsv channels defined in 0.0 .. 1.0 interval
float fract(float x) { return x - int(x); }
float mix(float a, float b, float t) { return a + (b - a) * t; }
float step(float e, float x) { return x < e ? 0.0 : 1.0; }

float *hsv2rgb(float h, float s, float b, float *rgb)
{
  // h = hue; s = saturation; b = brightness
  rgb[0] = s * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);
  rgb[1] = s * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);
  rgb[2] = s * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);
  return rgb;
}

void setup()
{
  // Serial port for debugging
  Serial.begin(115200U);
  delay(500);
  Serial.println();
  Serial.println("[  INIT  ] Hichi IR TTL 1.0");
  initStage++;

  Serial.printf("[  INIT  ] %s (Rev. %d)   Cores: %d   CpuFreq: %uMHz   Flash: %uKiB   Heap: %uKiB   MAC: %s   Sdk: %s\n",
                ESP.getChipModel(),
                ESP.getChipRevision(),
                ESP.getChipCores(),
                ESP.getCpuFreqMHz(),
                ESP.getFlashChipSize() / 1024,
                ESP.getHeapSize() / 1024,
                WiFi.macAddress().c_str(),
                ESP.getSdkVersion());
  initStage++;

  // Serial port for communication with the IR-Transceiver
  Serial1.begin(300u, SERIAL_7E1, RX1, TX1);
  Serial.print("[  INIT  ] IR-Transceiver enabled\n");

  // all done
  Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
}

void loop()
{
  timestamp = micros();

  // 50ms Tasks
  if ((timestamp - task50msTimer) > 50000L)
  {
    task50msTimer = timestamp;
    counter50ms++;

#ifdef BOARD_HAS_NEOPIXEL
    hsv2rgb(float(counter50ms % 60) / 60., 1.0, 1.0, rgbCol);
    neopixelWrite(ledBuiltin, rgbCol[0] * 32, rgbCol[1] * 32, rgbCol[2] * 32);
#endif

    return;
  }

  // 1s Tasks
  if ((timestamp - task1sTimer) > 1000000L)
  {
    task1sTimer = timestamp;
    counter1s++;

    Serial1.print("/?!\r\n");

    int received = Serial1.read(rxBuffer, sizeof(rxBuffer) - 1);
    if (received > 0)
    {
      rxBuffer[received] = 0;
      Serial.printf("[   IR   ] %s\n", rxBuffer);
    }

    return;
  }
}
