/*
  RadioLib CC1101 Blocking Receive Example

  This example receives packets using CC1101 FSK radio module.
  To successfully receive data, the following settings have to be the same
  on both transmitter and receiver:
  - carrier frequency
  - bit rate
  - frequency deviation
  - sync word

  Using blocking receive is not recommended, as it will lead
  to significant amount of timeouts, inefficient use of processor
  time and can some miss packets!
  Instead, interrupt receive is recommended.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#cc1101

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
// include the library
#include <_m5Core2-only.h>
//#include <M5Unified.h>
#include <_viewController.h>
#include "built_on.h"

#include <RadioLib.h>

#define CORES3_MOSI 37
#define CORES3_MISO 35
#define CORES3_SCK  36
#define CORES3_CS   5
#define CORES3_IO2  10
#define CORES3_IO0  7

#define CORE2_MOSI  23
#define CORE2_MISO  38
#define CORE2_SCK    18
#define CORE2_CS   27
#define CORE2_IO2  19
#define CORE2_IO0  33

#if defined (ARDUINO_M5STACK_CORE2)

//#pragma message ("YELLOW -----------------------------------")
#define CORE_MOSI CORE2_MOSI
#define CORE_MISO CORE2_MISO
#define CORE_SCK  CORE2_SCK
#define CORE_CS   CORE2_CS
#define CORE_IO2  CORE2_IO2
#define CORE_IO0  CORE2_IO0

#elif defined (ARDUINO_M5STACK_CORES3)

//#pragma message ("BLACK -----------------------------------")
#define CORE_MOSI CORES3_MOSI
#define CORE_MISO CORES3_MISO
#define CORE_SCK  CORES3_SCK
#define CORE_CS   CORES3_CS
#define CORE_IO2  CORES3_IO2
#define CORE_IO0  CORES3_IO0

#else
#error "no such processor"
#endif

// CC1101 has the following connections:
// CS pin:    10
// GDO0 pin:  2
// RST pin:   unused
// GDO2 pin:  3

CC1101 radio = new Module(CORE_CS, CORE_IO0, RADIOLIB_NC, CORE_IO2);

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

void setup() {
    _setup_M5();

    Serial.begin(115200);
    delay(1000);

  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  Serial.print(F("[CC1101] Waiting for incoming transmission ... "));
    _loop_M5();

  // you can receive data as an Arduino String
  String str;
  int state = radio.receive(str);

  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[CC1101] Data:\t\t"));
    Serial.println(str);

    // print RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[CC1101] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print LQI (Link Quality Indicator)
    // of the last received packet, lower is better
    Serial.print(F("[CC1101] LQI:\t\t"));
    Serial.println(radio.getLQI());

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
}
