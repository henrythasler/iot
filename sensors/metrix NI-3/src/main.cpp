#include <Arduino.h>

#define REED_INPUT (2)

uint32_t initStage = 0;
uint64_t timestamp = 0;

bool triggered = false;
static uint64_t debounceTimeout = 300;
uint64_t debounceTimer = 0;
int32_t triggerCounter = 0;

uint64_t lastIsr = 0;
uint64_t task100msTimer = 0;

void onInterrupt()
{
  uint64_t timestamp = millis();

  if ((timestamp - lastIsr) > 150)
  {
    digitalWrite(LED_BUILTIN, 1);
    triggered = true;
    debounceTimer = timestamp;
    lastIsr = timestamp;
  }
}


size_t print64(uint64_t number, int base)
{
    size_t n = 0;
    unsigned char buf[64];
    uint8_t i = 0;

    if (number == 0)
    {
        n += Serial.print((char)'0');
        return n;
    }

    if (base < 2) base = 2;
    else if (base > 16) base = 16;

    while (number > 0)
    {
        uint64_t q = number/base;
        buf[i++] = number - q*base;
        number = q;
    }

    for (; i > 0; i--)
    n += Serial.write((char) (buf[i - 1] < 10 ?
    '0' + buf[i - 1] :
    'A' + buf[i - 1] - 10));

    return n;
}

void setup()
{
  // put your setup code here, to run once:
  timestamp = millis();
  pinMode(LED_BUILTIN, OUTPUT); // Set Pin 13 as OUTPUT
  pinMode(2, INPUT_PULLUP);

  // Setup serial connection for debugging
  Serial.begin(115200U);
  delay(100);
  Serial.println();
  Serial.println("[  INIT  ] Begin");
  initStage++;

  attachInterrupt(digitalPinToInterrupt(REED_INPUT), onInterrupt, FALLING); //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)
}

void loop()
{
  timestamp = millis();

  if (triggered && ((timestamp - debounceTimer) > debounceTimeout))
  {
    if (digitalRead(REED_INPUT))
    {
      triggerCounter++;
      // Serial.print("[  ISR   ] Trigger! ");
      Serial.print("{\"counter\":");Serial.print(triggerCounter);
      Serial.print(",\"timestamp\":");print64(timestamp, 10);
      Serial.println("}");
    }

    triggered = false;
    return;
  }

  // 100ms Task
  if ((timestamp - task100msTimer) > 100L)
  {
    task100msTimer = timestamp;
    digitalWrite(LED_BUILTIN, 0);
    return;
  }
}
