#include <Arduino.h>

// Wifi and MQTT dependencies
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

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Environment sensor includes and defines
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define BME280_ADDR (0x76)
#define BME280_PIN_VCC (2)
#define PRESSURE_MEASUREMENT_CALIBRATION (6000)
#define SEALEVEL_PRESSURE (1013.25)
#define SEALEVEL_PRESSURE_CALIBRATION (9.65)

Adafruit_BME280 bme; // use I2C

// dynamic feature availability
bool environmentSensorAvailable = false;
bool mqttAvailable = false;

uint32_t initStage = 0;

uint32_t timestamp;
uint32_t task100msTimer = 0;
uint32_t task1sTimer = 0;
uint32_t task30sTimer = 0;

// sensor data
float currentTemperatureCelsius;
float currentHumidityPercent;
float currentPressurePascal;

// multi-purpose static bytearray
uint8_t txBuffer[256];
uint8_t rxBuffer[256];


void reconnect()
{
  Serial.print("[  MQTT  ] Attempting MQTT connection... ");
  if (mqttClient.connect(WiFi.hostname().c_str()))
  {
    Serial.println("connected");
    mqttAvailable = true;
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    mqttAvailable = false;
  }
}

void setup()
{
    // Setup serial connection for debugging
    Serial.begin(115200U);
    delay(500);
    Serial.println();
    Serial.println("[  INIT  ] Begin");
    initStage++;

    // connect to your local wi-fi network
    Serial.printf("[  INIT  ] Connecting to Wifi '%s'", secrets.wifiSsid);
    WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);

    // check wi-fi is connected to wi-fi network
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.printf(" connected! (IP=%s)\n", WiFi.localIP().toString().c_str());

    // Power-On Environment Sensor
    pinMode(BME280_PIN_VCC, OUTPUT);
    digitalWrite(BME280_PIN_VCC, HIGH);
    delay(5); // wait for BMW280 to power up. Takes around 2ms.
    initStage++;

    // Initialize Environment Sensor
    if (bme.begin(BME280_ADDR, &Wire)) // use custom Wire-Instance to avoid interference with other libraries.
    {
        initStage++;
        environmentSensorAvailable = true;
        Serial.printf("[  INIT  ] found BME280 environment sensor (ID 0x%02X)\n", bme.sensorID());
    }

    if (environmentSensorAvailable)
    {
        initStage++;
        // Setup Environment Sensor
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BME280::SAMPLING_X16,    /* Temp. oversampling */
                        Adafruit_BME280::SAMPLING_X16,    /* Hum. oversampling */
                        Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BME280::FILTER_X16,      /* Filtering. */
                        Adafruit_BME280::STANDBY_MS_500); /* Standby time. */
        Serial.println("[  INIT  ] BME280 setup done");
    }
    else
    {
        Serial.println("[ ERROR  ] Could not find a BME280 sensor, check wiring!");
    }
    initStage++;

    Serial.println("[  INIT  ] Connecting to MQTT-Server...");
    mqttClient.setServer(secrets.mqttBroker, 1883);
    initStage++;

    Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
}

void loop()
{
    timestamp = millis();

    // 100ms Tasks
    if ((timestamp - task100msTimer) > 100L)
    {
        task100msTimer = timestamp;

        if (!mqttClient.connected())
        {
            reconnect();
        }
        mqttClient.loop();
        return;
    }

    // 1s Tasks
    if ((timestamp - task1sTimer) > 1000L)
    {
        task1sTimer = timestamp;
        return;
    }

    // 30s Tasks
    if ((timestamp - task30sTimer) > 30000L)
    {
        task30sTimer = timestamp;

        if (environmentSensorAvailable)
        {
            // read current measurements
            currentTemperatureCelsius = bme.readTemperature();
            currentHumidityPercent = bme.readHumidity();
            currentPressurePascal = bme.readPressure() + PRESSURE_MEASUREMENT_CALIBRATION;

            // memory state
            Serial.printf("[ STATUS ] Free: %u KiB (%u KiB)  Temp=%.1f \u00b0C  Hum=%.0f %%  Press=%.0f hPa\n",
                          ESP.getFreeHeap() / 1024,
                          ESP.getMaxFreeBlockSize() / 1024,
                          currentTemperatureCelsius,
                          currentHumidityPercent,
                          currentPressurePascal / 100.0);

            if (mqttAvailable)
            {
                int32_t len = 0;
                len = snprintf((char *)txBuffer, sizeof(txBuffer), "{\"temp\":{\"value\":%.1f,\"unit\":\"\u00b0C\"},\"hum\":{\"value\":%.0f,\"unit\":\"%%\"},\"press\":{\"value\":%.0f,\"unit\":\"hPa\"}}",
                               currentTemperatureCelsius,
                               currentHumidityPercent,
                               currentPressurePascal / 100.0);
                mqttClient.publish("home/inside/livingroom", txBuffer, len);
            }
        }
        return;
    }
}