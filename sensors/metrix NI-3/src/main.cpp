#include <Arduino.h>

#define REED_INPUT (2)

uint64_t timestamp = 0;

static int32_t debounceThresold = 2; // need that many consecutive readings for an edge
int32_t debounceCounter = 0;
int32_t inputState = -1;

int64_t pulseCounter = 0;

uint64_t task100msTimer = 0;
uint64_t task1sTimer = 0;

int32_t inputPin = LOW;

size_t print64(uint64_t number, int base = 10);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT_PULLUP);

  // Setup serial connection for debugging
  Serial.begin(115200U);
  delay(100);
  Serial.println("Init ok");
}

void loop()
{
  timestamp = millis();

  // 1s Task
  if ((timestamp - task1sTimer) > 1000L)
  {
    task1sTimer = timestamp;
    // Serial.print((inputPin == HIGH) ? 'X' : '.');

    if ((inputState == LOW) && (inputPin == HIGH))
    {
      debounceCounter++;
      if (debounceCounter >= debounceThresold)
      {
        inputState = HIGH;
        debounceCounter = 0;
      }
    }
    else if ((inputState == HIGH) && (inputPin == LOW))
    {
      debounceCounter++;
      if (debounceCounter >= debounceThresold)
      {
        inputState = LOW;
        debounceCounter = 0;
        pulseCounter++;

        Serial.print("{\"pulseCounter\":");
        print64(pulseCounter);
        Serial.print(",\"timestamp\":");
        print64(timestamp);
        Serial.println("}");
      }
    }
    return;
  }

  // 100ms Task
  if ((timestamp - task100msTimer) > 100L)
  {
    task100msTimer = timestamp;
    inputPin = digitalRead(REED_INPUT);
    digitalWrite(LED_BUILTIN, HIGH - inputPin);
    return;
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

  if (base < 2)
    base = 2;
  else if (base > 16)
    base = 16;

  while (number > 0)
  {
    uint64_t q = number / base;
    buf[i++] = number - q * base;
    number = q;
  }

  for (; i > 0; i--)
    n += Serial.write((char)(buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));

  return n;
}