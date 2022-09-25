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
uint32_t timestamp = 0;

// sensor data
float currentTemperatureCelsius;
float currentHumidityPercent;
float currentPressurePascal;

// multi-purpose static bytearray
uint8_t txBuffer[256];

void reconnect()
{
    Serial.print("[  MQTT  ] Connecting to MQTT-Broker... ");
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
    timestamp = millis();

    // Setup serial connection for debugging
    Serial.begin(115200U);
    delay(100);
    Serial.println();
    Serial.println("[  INIT  ] Begin");
    initStage++;

    // Power-On Environment Sensor
    pinMode(BME280_PIN_VCC, OUTPUT);
    digitalWrite(BME280_PIN_VCC, HIGH);
    delay(5); // wait for BMW280 to power up. Takes around 2ms.
    initStage++;

    // Initialize Environment Sensor
    if (bme.begin(BME280_ADDR, &Wire))
    {
        initStage++;
        environmentSensorAvailable = true;

        // Setup Environment Sensor
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode: Continuous sampling*/
                        Adafruit_BME280::SAMPLING_X16,    /* Temp. oversampling: Set to max to reduce noise */
                        Adafruit_BME280::SAMPLING_X16,    /* Hum. oversampling: Set to max to reduce noise */
                        Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling: Set to max to reduce noise */
                        Adafruit_BME280::FILTER_X16,      /* Filtering: Set to max to reduce noise */
                        Adafruit_BME280::STANDBY_MS_10);  /* Standby time: Use High-Frequency sampling with maximum filter to minimize noise*/
        Serial.printf("[  INIT  ] found BME280 environment sensor (ID 0x%02X)\n", bme.sensorID());
    }
    else
    {
        Serial.println("[ ERROR  ] Could not find a BME280 sensor, check wiring!");
    }

    // connect to your local wi-fi network
    Serial.printf("[  INIT  ] Connecting to Wifi '%s'", secrets.wifiSsid);
    WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);

    // check wi-fi is connected to wi-fi network
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(250);
        Serial.print(".");
    }
    Serial.printf(" connected! (IP=%s)\n", WiFi.localIP().toString().c_str());

    // MQTT-Connection
    mqttClient.setServer(secrets.mqttBroker, 1883);
    reconnect();
    initStage++;

    Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);

    if (environmentSensorAvailable)
    {
        // read current measurements
        currentTemperatureCelsius = bme.readTemperature();
        currentHumidityPercent = bme.readHumidity();
        currentPressurePascal = bme.readPressure() + PRESSURE_MEASUREMENT_CALIBRATION;

        // memory state
        Serial.printf("[ STATUS ] Free: %u KiB (%u KiB)  Temp=%.1f \u00b0C  Hum=%.0f %%  Press=%.1f hPa\n",
                      ESP.getFreeHeap() / 1024,
                      ESP.getMaxFreeBlockSize() / 1024,
                      currentTemperatureCelsius,
                      currentHumidityPercent,
                      currentPressurePascal / 100.0);

        if (mqttAvailable)
        {
            int32_t len = snprintf((char *)txBuffer, sizeof(txBuffer), "{\"temp\":{\"value\":%.1f,\"unit\":\"\u00b0C\"},\"hum\":{\"value\":%.0f,\"unit\":\"%%\"},\"press\":{\"value\":%.1f,\"unit\":\"hPa\"}}",
                           currentTemperatureCelsius,
                           currentHumidityPercent,
                           currentPressurePascal / 100.0);
            mqttClient.publish("home/inside/livingroom", txBuffer, len, true);
        }
    }
    mqttClient.loop();
    Serial.printf("[ STATUS ] Completed in %ums. Entering Deep-Sleep-Mode...\n", millis() - timestamp);
    ESP.deepSleep(60 * 1000 * 1000);
}

void loop(){}