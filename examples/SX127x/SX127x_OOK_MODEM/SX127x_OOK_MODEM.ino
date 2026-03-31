/*
  RadioLib SX127x FSK Modem Example

  This example shows how to use FSK modem in SX127x chips.

  NOTE: The sketch below is just a guide on how to use
        FSK modem, so this code should not be run directly!
        Instead, modify the other examples to use FSK
        modem and use the appropriate configuration
        methods.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---fsk-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/
#include <M5Unified.h>
#include <SPI.h>
#include <Arduino.h>

// include the library
#include <RadioLib.h>

#define LORA_CS   27
#define LORA_IRQ  35   // aka DIO0 ?
#define LORA_RST  -1
#define LORA_BUSY -1
#define LORA_SCK  18
#define LORA_MISO 38
#define LORA_MOSI 23

const int DIO2_DATA = 25;  //DIO2


// Create a custom SPIClass instance (e.g., using VSPI or HSPI on ESP32)
// The specific pins used are defined by the begin() call on the instance
//SPIClass customSPI(VSPI); 

// Create custom SPISettings (e.g., 2 MHz speed)
//SPISettings customSPISettings(2000000, MSBFIRST, SPI_MODE0);

// Instantiate the Module with custom SPI parameters
                // Module(     cs,      irq,      rst,      gpio,      &spi, spiSettings
SX1278 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY); //, customSPI, customSPISettings);

#if 0
// SX1278 has the following connections:
#define NSS  27
#define DIO0   -1
#define REESET -1
#define DIO1   -1
SX1278 radio = new Module(NSS, DIO0, REESET, DIO1);
#endif

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

volatile bool bIsrBit;
volatile bool bLastBit;
uint32_t isrCnt;


// Function called every time a bit is received (interrupt)
void ISR_readbit(void) 
{
  // Read the raw bit from the DIO2 pin
  bIsrBit = digitalRead(DIO2_DATA);
  isrCnt++;
}
	
void setup() 
{

  pinMode(DIO2_DATA, INPUT);
  
  Serial.begin(115200);
  M5.begin();

  // reset radio by dropping power.
  M5.Power.setExtPower(false);
  delay(500);
  M5.Power.setExtPower(true);

  
  // initialize SX1278 FSK modem with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  
  int state = radio.beginFSK();
  if (state == RADIOLIB_ERR_NONE) 
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // if needed, you can switch between LoRa and FSK modes
  // radio.begin()       start LoRa mode (and disable FSK)
  
  // the following settings can also
  // be modified at run-time
  state = radio.setFrequency(433.911 - .002300);   // freq for wx?
  state = radio.setBitRate(.5);
  
  state = radio.setFrequencyDeviation(10.0);
  state = radio.setRxBandwidth(8.0);
  state = radio.setOutputPower(2.0); 	//lowest pwr.
  state = radio.setCurrentLimit(100);
  
  state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
  uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};

  state = radio.setSyncWord(syncWord, 8);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to set configuration, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // FSK modulation can be changed to OOK
  // NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
  //       Also, data shaping changes from Gaussian filter to
  //       simple filter with cutoff frequency. Make sure to call
  //       setDataShapingOOK() to set the correct shaping!
  
  state = radio.setOOK(true);
  state = radio.setDataShapingOOK(2);  // no shaping 
  if (state != RADIOLIB_ERR_NONE) 
  {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // ------------------------------------------------
  
  /*
	byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
					  0x89, 0xAB, 0xCD, 0xEF};
	int state = radio.transmit(byteArr, 8);
  */

  const char *test = "01234567890123456789012345678901234567890123456789";  

  for (int i = 0; i < 5; i++)
  {
	  // transmit OOK packet
	  //state = radio.transmit("Hello World!");
 	  state = radio.transmit( test, 0);
	  if (state == RADIOLIB_ERR_NONE) 
	  {
	    Serial.printf("[SX1278] Packet %d transmitted successfully!\n", i);
	  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
	    Serial.println(F("[SX1278] Packet too long!"));
	  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
	    Serial.println(F("[SX1278] Timed out while transmitting!"));
	  } else {
	    Serial.println(F("[SX1278] Failed to transmit packet, code "));
	    Serial.println(state);
	  }
	  delay(500);
  }

  Serial.printf("starting to loop\n");
 
   attachInterrupt(DIO2_DATA, ISR_readbit, CHANGE);
   state = radio.directMode();

}


uint32_t oldIsr = -1;

void loop() {
  // FSK modem can use the same transmit/receive methods
  // as the LoRa modem, even their interrupt-driven versions
  // NOTE: FSK modem maximum packet length is 63 bytes!
  int state;


  while (true)
  {
	 if (oldIsr !=  isrCnt)
	 {
	 	Serial.printf("count = %d\n", isrCnt);
	 	oldIsr = isrCnt;
	 }
	 
	 if (bIsrBit != bLastBit)
	 {
	  	bLastBit = bIsrBit;
		Serial.printf("hi %d\n", bLastBit);
	 }
  }; 	
#if 0
  // FSK modem has built-in address filtering system
  // it can be enabled by setting node address, broadcast
  // address, or both
  //
  // to transmit packet to a particular address,
  // use the following methods:
  //
  // radio.transmit("Hello World!", address);
  // radio.startTransmit("Hello World!", address);

  // set node address to 0x02
  state = radio.setNodeAddress(0x02);
  // set broadcast address to 0xFF
  state = radio.setBroadcastAddress(0xFF);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Unable to set address filter, code "));
    Serial.println(state);
  }

  // address filtering can also be disabled
  // NOTE: calling this method will also erase previously set
  //       node and broadcast address
  /*
    state = radio.disableAddressFiltering();
    if (state != RADIOLIB_ERR_NONE) {
      Serial.println(F("Unable to remove address filter, code "));
    }
  */

  // FSK modem supports direct data transmission
  // in this mode, SX127x directly transmits any data
  // sent to DIO1 (data) and DIO2 (clock)

  // activate direct mode transmitter
  state = radio.transmitDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Unable to start direct transmission mode, code "));
    Serial.println(state);
  }

  // using the direct mode, it is possible to transmit
  // FM notes with Arduino tone() function

  // it is recommended to set data shaping to 0
  // (no shaping) when transmitting audio
  state = radio.setDataShaping(0.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Unable to set data shaping, code "));
    Serial.println(state);
  }

  // transmit FM tone at 1000 Hz for 1 second, then 500 Hz for 1 second
  // (DIO2 is connected to Arduino pin 4)
  // Note: tone() function is not available on Arduino Due and CubeCell
  //       on these platforms, the following will do nothing
  #if !defined(RADIOLIB_TONE_UNSUPPORTED)
  tone(4, 1000);
  delay(1000);
  tone(4, 500);
  delay(1000);
  noTone(4);
  #endif

  // NOTE: after calling transmitDirect(), SX127x will start
  // transmitting immediately! This signal can jam other
  // devices at the same frequency, it is up to the user
  // to disable it with standby() method!

  // direct mode transmissions can also be received
  // as bit stream on DIO1 (data) and DIO2 (clock)
  state = radio.receiveDirect();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Unable to start direct reception mode, code "));
    Serial.println(state);
  }

  // NOTE: you will not be able to send or receive packets
  // while direct mode is active! to deactivate it, call method
  // radio.packetMode()

  #endif
}
