/*
 * RadioLib CC1101 Transmit with Interrupts Example
 *
 * This example transmits packets using CC1101 FSK radio module.
 * Once a packet is transmitted, an interrupt is triggered.
 * Each packet contains up to 255 bytes of data with some limitations (https://github.com/jgromes/RadioLib/discussions/1138), in the form of:
 * - Arduino String
 * - null-terminated char array (C-string)
 * - arbitrary binary data (byte array)
 *
 * For default module settings, see the wiki page
 * https://github.com/jgromes/RadioLib/wiki/Default-configuration#cc1101
 *
 * For full API reference, see the GitHub Pages
 * https://jgromes.github.io/RadioLib/
 */

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
 * Radio radio = new RadioModule();
 */

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool bTxDone = false;
volatile bool bAllowNextTx = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void onIRQtx(void)
{
    // we sent a packet, set the flag
    bTxDone = true;
}


void setup()
{
    _setup_M5();
    
    Serial.begin(115200);
	delay(1000);
	

    // initialize CC1101 with default settings
    Serial.print(F("[CC1101] Initializing ... "));
    int state = radio.beginFSK4();

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);

        while (true)
            delay(10);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setPacketSentAction(onIRQtx);

    // start transmitting the first packet
    Serial.print(F("[CC1101] Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 255 characters long
    transmissionState = radio.startTransmit("Hello World!");

    // you can also transmit byte array up to 255 bytes long
    // When transmitting more than 64 bytes startTransmit blocks to refill the FIFO.
    // Blocking ceases once the last bytes have been placed in the FIFO
    /*
     * byte byteArr[] = {0x01, 0x23, 0x45, 0x56,
     *                  0x78, 0xAB, 0xCD, 0xEF};
     * state = radio.startTransmit(byteArr, 8);
     */
}


// counter to keep track of transmitted packets
int count = 0;
void loop()
{
	static uint32_t snore;
    static uint32_t cnt;
    static uint32_t temp;
	uint32_t milli;
	milli = millis();
	
    _loop_M5();

    // check if the previous transmission finished
    if (bTxDone && !bAllowNextTx)
    {
    	while (true)
    	{
    		uint8_t cnt = radio.getTxFifoCount();
    		Serial.printf("drain fifo = %d\n", cnt);
    		if (!cnt) break;
    	}
    	

        if (transmissionState == RADIOLIB_ERR_NONE)
        {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()

			// clean up after transmission is finished
			// this will ensure transmitter is disabled,
			// RF switch is powered down etc.
			radio.finishTransmit();

			bAllowNextTx = true;
			
        }
        else
        {
            Serial.printf("transmissionState failed = 0x%X\n", transmissionState);
            assert(0);
        }
	}

	
    // wait a second before transmitting again
	if (snore < milli && bAllowNextTx)
	{
		snore = milli + 10000;

        // you can transmit C-string or Arduino string up to
        // 255 characters long
        String str = "01234567890123456789  #" + String(count++);
        transmissionState = radio.startTransmit(str);

		// reset flag
		bTxDone = false;

        static uint32_t tx_cnt;
        Serial.println(str);
        
        _cprintf(_GREEN, 6, "tx cnt = %d", ++tx_cnt);

		uint8_t cnt = radio.getTxFifoCount();
		Serial.printf("stuff fifo = %d\n", cnt);

        // you can also transmit byte array up to 255 bytes long with limitations https://github.com/jgromes/RadioLib/discussions/1138
        /*
         * byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
         *                  0x89, 0xAB, 0xCD, 0xEF};
         * int state = radio.startTransmit(byteArr, 8);
         */
    }
    else
    {
  		_cprintf(_YELLOW, 7, "%d vs %d\n", snore, milli);
    }
    
	_cprintf(_YELLOW, 7, "%d vs %d\n", snore, milli);
	_cprintf(_RED, 8, "c= %d t=%d", cnt++, milli);
}
