/*
 * RadioLib CC1101 Receive with Interrupts Example
 *
 * This example listens for FSK transmissions and tries to
 * receive them. Once a packet is received, an interrupt is
 * triggered.
 *
 * To successfully receive data, the following settings have to be the same
 * on both transmitter and receiver:
 * - carrier frequency
 * - bit rate
 * - frequency deviation
 * - sync word
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

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

 SemaphoreHandle_t sem_DATA_READY = xSemaphoreCreateBinary();

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!


#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void onIRQrx(void)
{
    // we got a packet, set the flag
    receivedFlag = true;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 	xSemaphoreGiveFromISR( sem_DATA_READY, &xHigherPriorityTaskWoken );

	// wake up task that need it.
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//-------------------------------------------------------------

void setup()
{
    _setup_M5();

    Serial.begin(115200);
    delay(1000);


    // initialize CC1101 with default settings
    Serial.print(F("[CC1101] Initializing ... "));
    int state = radio.begin();

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
    // when new packet is received
    radio.setPacketReceivedAction(onIRQrx);

    // start listening for packets
    Serial.print(F("[CC1101] Starting to listen ... "));
    state = radio.startReceive();

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

    // if needed, 'listen' mode can be disabled by calling
    // any of the following methods:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.readData();
    _lclear();
    _cprintf(_WHITE, 0, "%s", built_on);
    _loop_M5();
    
}

uint32_t timer;

//-------------------------------------------------------------

void loop()
{
	static uint32_t rx_cnt;
	uint8_t cline = 0;
	
	char msg[50];
    _loop_M5();

	int ret1 = xSemaphoreTake( sem_DATA_READY, pdMS_TO_TICKS(30000));
	if (ret1 == pdTRUE)
	{

        byte byteArr[1000];
        //memset(byteArr, 'U', sizeof(byteArr));

        int numBytes = radio.getPacketLength(true);
        Serial.printf("xxxx len = %d\n", numBytes);

        int state = radio.readData(byteArr, numBytes);
        byteArr[numBytes]= 0;
        
        if (state == RADIOLIB_ERR_NONE)
        {
            // packet was successfully received
            Serial.println(F("[CC1101] Received packet!"));

            // print data of the packet
            Serial.print(F("[CC1101] Data:\t\t"));
			Serial.printf(">>> %s\n", (char *)byteArr);

			sprintf(msg,"rx = %d len=%d", ++rx_cnt, numBytes);
 			Serial.printf("%s\n", msg);
 			_cprintf(_GREEN, ++cline, "%s\n", msg);
			
            // print RSSI (Received Signal Strength Indicator)
            // of the last received packet
            
	        sprintf(msg, "RSSI: %5.1f", radio.getRSSI());
 			Serial.printf("%s\n", msg);
 			_cprintf(_GREEN, ++cline, "%s\n", msg);
 			
            // print LQI (Link Quality Indicator)
            // of the last received packet, lower is better
            
	        sprintf(msg, "LQI: %5.1f", radio.getLQI());
 			Serial.printf("%s\n", msg);
 			_cprintf(_GREEN, ++cline, "%s\n", msg);
 
        }
        else if (state == RADIOLIB_ERR_CRC_MISMATCH)
        {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));

        }
        else
        {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);

        }

        // put module back to listen mode
        radio.startReceive();
    }
}
