#include "Arduino.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "wifiConfig.h"

void OverTheAirUpdate()
{
  AsyncWebServer server(80);
  if(!WiFi.softAPConfig(LOCAL_AP_ADD, GATEWAY, SUBNET)){
    Serial.println("STA failed to configure");
  }
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print(F("To upload, go to:"));
  Serial.print(WiFi.softAPIP());
  Serial.println(F("/update\n"));
  //Serial << F("To upload, go to: ") << WiFi.softAPIP() << F("/update\n");
  
  String mensaje = "<p>To upload new software, please go to: " + LOCAL_AP_ADD.toString() + "/update</p>";
  server.on("/", HTTP_GET, [mensaje](AsyncWebServerRequest * request) {
    request->send(200, "text/html", mensaje);
  });
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  while(1){
    delay(1);
  }
  
}