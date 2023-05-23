// CONFIGURATION FILE FOR WIFI
#include <Arduino.h>
#include <WiFi.h>

const char* host = "esp32";
const char* ssid = "MidiESP OTA";
const char* password = NULL;

IPAddress LOCAL_AP_ADD(192,168,5,1);
IPAddress GATEWAY(192,168,5,1);
IPAddress SUBNET(255,255,255,0);