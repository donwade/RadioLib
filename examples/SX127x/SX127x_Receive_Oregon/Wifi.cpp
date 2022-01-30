#include <time.h>
#include <Udp.h>
#include <WiFi.h>
#include "wificonfig.h"

#ifndef ssid
	#error "you forgot ssid define in wificonfig.h"
#endif
#ifndef password
    #error "you forgot password define in wificonfig.h"
#endif

// or
//const char* ssid       = "whitehouse";
//const char* password   = "joebiden341";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600 * 5;
const int   daylightOffset_sec = 3600;

//---------------------------------------

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
//---------------------------------------

void setupWiFi()
{

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.printf(" CONNECTED to %s\n", ssid);

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

