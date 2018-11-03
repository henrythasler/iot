#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

MPU9250 IMU(Wire, 0x68);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
    if(type == WS_EVT_CONNECT){
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    //client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG){
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if(info->opcode == WS_TEXT)
        client->text("I got your text message");
      else
        client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if((info->index + len) == info->len){
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}


const char* ssid = "Thasler";
#include "password.h"   // const char* password = "TOPSECRET"
const char* hostName = "car";
const char* http_username = "admin";
const char* http_password = "admin";

void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  WiFi.hostname(hostName);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(hostName);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("STA: Failed!\n");
    WiFi.disconnect(false);
    delay(1000);
    WiFi.begin(ssid, password);
  }

  //Send OTA events to the browser
  ArduinoOTA.onStart([]() { events.send("Update Start", "ota"); });
  ArduinoOTA.onEnd([]() { events.send("Update End", "ota"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char p[32];
    sprintf(p, "Progress: %u%%\n", (progress/(total/100)));
    events.send(p, "ota");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if(error == OTA_AUTH_ERROR) events.send("Auth Failed", "ota");
    else if(error == OTA_BEGIN_ERROR) events.send("Begin Failed", "ota");
    else if(error == OTA_CONNECT_ERROR) events.send("Connect Failed", "ota");
    else if(error == OTA_RECEIVE_ERROR) events.send("Recieve Failed", "ota");
    else if(error == OTA_END_ERROR) events.send("End Failed", "ota");
  });
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();

  MDNS.addService("http","tcp",80);

  SPIFFS.begin();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);

  
  server.addHandler(new SPIFFSEditor(http_username,http_password)); // access with http://<IP-ADDRESS>/edit

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");;

  server.on("/*.gz", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html.gz", "text/html");
        response->addHeader("Content-Encoding","gzip");
        request->send(response);
  });  


  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });
  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index)
      Serial.printf("UploadStart: %s\n", filename.c_str());
    Serial.printf("%s", (const char*)data);
    if(final)
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });
  server.begin();
}


#define FILTER 20

float yaw[FILTER], pitch, roll;
float ax, ay, az, gx, gy, gz, mx, my, mz;
float a12, a22, a31, a32, a33;
float lin_ax, lin_ay, lin_az;
unsigned long Now, lastUpdate;
unsigned long timeBase_2s = 0, timeBase_17ms = 0;
float deltat;

bool calibrated = true;
bool initialized = false;
float hxb=-11.9389, hxs=1.0558, hyb=64.6017, hys=0.9558, hzb=4.3337, hzs=0.9935;

unsigned int cnt = 0;

void loop(){
  ArduinoOTA.handle();

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  timeBase_2s += (Now - lastUpdate);
  timeBase_17ms += (Now - lastUpdate);
  lastUpdate = Now;

  if(!initialized) 
  {
    if(timeBase_2s > 2000000) 
    {
      // start communication with IMU
      int status = IMU.begin();
      if (status < 0)
      {
        Serial.println("[init] - IMU initialization error");
      }
      else 
      {
        Serial.println("[init] - IMU initialization successful");
        initialized = true;
        // setting the accelerometer full scale range to +/-8G
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);

        // set calibration values
        if (calibrated)
        {
          IMU.setMagCalX(hxb, hxs);
          IMU.setMagCalY(hyb, hys);
          IMU.setMagCalZ(hzb, hzs);
          Serial.println("[init] - IMU calibration done");
        }  
      }
    }
  }
  
  else
  {
    if (!calibrated)
    {
        Serial.println("Starting calibration. Move the sensor slowly in every direction.");
        int status = IMU.calibrateMag();
        Serial.println(status);
        Serial.println("Calibration finished");

        hxb = IMU.getMagBiasX_uT();
        hxs = IMU.getMagScaleFactorX();
        hyb = IMU.getMagBiasY_uT();
        hys = IMU.getMagScaleFactorY();
        hzb = IMU.getMagBiasZ_uT();
        hzs = IMU.getMagScaleFactorZ();

        Serial.print("hxb=");
        Serial.print(hxb, 4);
        Serial.print(", hxs=");
        Serial.print(hxs, 4);
        Serial.print(", hyb=");
        Serial.print(hyb, 4);
        Serial.print(", hys=");
        Serial.print(hys, 4);
        Serial.print(", hzb=");
        Serial.print(hzb, 4);
        Serial.print(", hzs=");
        Serial.println(hzs, 4);

        IMU.setMagCalX(hxb, hxs);
        IMU.setMagCalY(hyb, hys);
        IMU.setMagCalZ(hzb, hzs);
        calibrated = true;
    }
    else
    {
      IMU.readSensor();
      // read the sensor according to right-hand-rule
        ax = -IMU.getAccelY_mss() / 9.81;
        ay = -IMU.getAccelX_mss() / 9.81;
        az = IMU.getAccelZ_mss() / 9.81;

        gx = IMU.getGyroY_rads();
        gy = IMU.getGyroX_rads();
        gz = -IMU.getGyroZ_rads();

        // X and Y axis are switched;
        mx = IMU.getMagY_uT();
        my = IMU.getMagX_uT();
        mz = -IMU.getMagZ_uT();

      // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
      // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
      // (+ up) of accelerometer and gyro! We have to make some allowance for this
      // orientationmismatch in feeding the output to the quaternion filter. For the
      // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
      // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
      // modified to allow any convenient orientation convention. This is ok by
      // aircraft orientation standards! Pass gyro rate as rad/s
      MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, deltat);
      //MahonyQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, deltat);

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
/*
      a12 = 2.0f * (*(getQ() + 1) * *(getQ() + 2) + *(getQ()) * *(getQ() + 3));
      a22 = *(getQ()) * *(getQ()) + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3);
      a31 = 2.0f * (*(getQ()) * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3));
      a32 = 2.0f * (*(getQ() + 1) * *(getQ() + 3) - *(getQ()) * *(getQ() + 2));
      a33 = *(getQ()) * *(getQ()) - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3);

      pitch = -asinf(a32);
      roll = atan2f(a31, a33);
      yaw[cnt] = atan2f(a12, a22);

//      pitch *= RAD_TO_DEG;
//      yaw[cnt] *= RAD_TO_DEG;
//      yaw[cnt] += 3.2f; // correction for Munich area
//        if (yaw[cnt] < 0)
//            yaw[cnt]+= 360.0f; // Ensure yaw stays between 0 and 360
//      roll *= RAD_TO_DEG;

      cnt = (cnt+1)%FILTER;
      
      lin_ax = ax + a31;
      lin_ay = ay + a32;
      lin_az = az - a33;
*/   
    }    
  }

  if(timeBase_2s > 2000000)
  {
    Serial.printf("[stat] - %3.0f Hz / %u clients\n", 1. / deltat, ws.count());
  }

  if (timeBase_17ms >= 25000) // transmit every 25ms (~40Hz)
  {
    if(ws.count() > 0) 
    {
/*      
      float txYaw=0;
      for(int x = 0; x<FILTER ;x++) {
          txYaw += yaw[x];
      }
      txYaw /= FILTER;
*/
      //ws.printfAll("{\"deltat\":%f, \"yaw\":%f, \"pitch\":%f, \"roll\":%f}", deltat, txYaw, pitch , roll);
      ws.printfAll("{\"deltat\":%f, \"x\":%f, \"y\":%f, \"z\":%f, \"w\":%f}", deltat, *(getQ() + 0), *(getQ() + 1) , *(getQ() + 2), *(getQ() + 3));
    }
  }

  if(timeBase_2s > 2000000) timeBase_2s = 0;  
  if(timeBase_17ms > 25000) timeBase_17ms = 0;  

}