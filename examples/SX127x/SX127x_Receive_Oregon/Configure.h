#ifndef __DEFINES_H__
#define __DEFINES_H__

#define ERASE_SD_ONPOWERUP 0

#define WIFI_AVAILABLE  0

#if WIFI_AVAILABLE
    #define MY_SSID "yourSSID"
    #define MY_PASSWORD "yourSSIDPASSWORD"
    #error "STOP: update your SSID/PASSWORD and delete this line"
#endif

#define SDCARD_LOGGING 1 //define this to be a one or a zero

#if SDCARD_LOGGING == 1
    #define JTAG_PRESENT 0
#elif SDCARD_LOGGING == 0
    #define JTAG_PRESENT 1
#else
    #error "STOP : define SDCARD_LOGGING to be 1 or 0 at line 16"
#endif

#endif  //__DEFINES_H__

