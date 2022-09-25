#include <Arduino.h>

#define REED_INPUT (2)

uint32_t initStage = 0;
uint64_t timestamp = 0;

bool triggered = false;
static int32_t debounceTimeout = 300;
int32_t debounceTimer = 0;
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
      Serial.print("{\"counter\":");Serial.print(triggerCounter);Serial.print("}");
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
