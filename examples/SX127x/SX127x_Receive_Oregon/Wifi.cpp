#include <time.h>
#include <Udp.h>
#include <WiFi.h>
#include "Configure.h"

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
#if WIFI_AVAILABLE

  //connect to WiFi
  Serial.printf("Connecting to %s ", MY_SSID);
  WiFi.begin(MY_SSID, MY_SSID);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.printf(" CONNECTED to %s\n", MY_SSID);

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#else
  Serial.printf("%s: Configured for no wifi\n", __FUNCTION__);
  delay(2000);
#endif

  printLocalTime();
}

