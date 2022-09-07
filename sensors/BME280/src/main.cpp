#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

/**
 * This file must contain a struct "secrets" with the following properties:
 *  const char *wifiSsid = "test" // WiFi AP-Name
 *  const char *wifiPassword = "1234"
 *  const char *ntpServer = "192.168.0.1";
 *  IPAddress mqttServer = IPAddress(192, 168, 0, 1); // MQTT-Broker
 **/
#include <secrets.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}