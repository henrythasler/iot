#include <Arduino.h>

uint32_t initStage = 0;
uint32_t timestamp = 0;

void onInterrupt()
{
  Serial.println("[  ISR   ] Trigger!");
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

  attachInterrupt(digitalPinToInterrupt(2), onInterrupt, RISING); //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)
}

void loop()
{
  timestamp = millis();
  // put your main code here, to run repeatedly:
  Serial.println(timestamp);

  digitalWrite(LED_BUILTIN, 1); // turn ON LED
  delay(500);                   // wait for 1 second
  digitalWrite(LED_BUILTIN, 0); // turn OFF LED
  delay(500);                   // wait for 1 second
}

