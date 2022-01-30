/*
   RadioLib SX127x Receive with Interrupts Example

   This example listens for LoRa transmissions and tries to
   receive them. Once a packet is received, an interrupt is
   triggered. To successfully receive data, the following
   settings have to be the same on both transmitter
   and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/
#include <rom/rtc.h>  // retrieve reset code interface.
#include "OregonV2.h"

#define TTGO
#define RADIOLIB_DEBUG
#define RADIOLIB_VERBOSE

#define DEAD_AIR_TIMEmS 2000  // if dead air is seen this long cough ... out current packet.

typedef struct {
    uint16_t bitCnt;
    int64_t arrivalTime;
    int32_t lateTime;
    uint64_t now;

    uint64_t shifter[2];
} COIN_TOSS;



// include the library
#include <RadioLib.h>

#define ERASE_SD_ONPOWERUP 0

#define GREEN_LED       25 // TTGO GPIO25
#define DUMB_433_INPUT  34   // OREGON DIRECT on chip

#define BOOT_MSG2 "using SX1726 lora chip output pin 32"
#define BOOT_MSG1 "simple 433 reciever connected to pin 34"

uint8_t rssiCalSetting;
uint8_t ookCalSetting;

#ifndef TTGO

// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
SX1278 radio = new Module(10, 2, 9, 3);

#else

#define SCK     5    // GPIO5  -- SCK
#define MISO    19   // GPIO19 -- MISO
#define MOSI    27   // GPIO27 -- MOSI
#define SS      18   // GPIO18 -- CS
#define DIO0    26   // GPIO26 -- IRQ (Interrupt Request)
#define DIO1    33   // GPIO33
#define RST     23   // TTGO 433 boards
#define DIO2    32   // GPIO32 real LORA chip output


SX1278 radio = new Module(SS, DIO0, RST, DIO1, DIO2);
//SX1278 radio = new Module(SS, DIO0, RST, DIO1); //dont bring in hardware for dio2

#endif

#define SCOPE_GPIO35 35  // avoid GPIO13 SD  !!!! AVOID GPIO2 its for SD CARD AND BOOTPGM!!!

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

static unsigned int clkChangeCtr = 0;
static unsigned int rssiChangeCtr = 0;

static QueueHandle_t DIO0queue = NULL;      // rssi etc.
static QueueHandle_t DIO1queue = NULL;      // rfm clock for direct mode
static QueueHandle_t DIO2Dataqueue = NULL;  // rfm data pin
bool volatile bCalInProgress = false;        // some cal operation in progress.

typedef struct AMessage
{
    int changeCnt;
    unsigned long absTime;
    unsigned long diffTime;
    bool bPinState;
    uint64_t debug;

    uint64_t headShifter[2];
    uint16_t headsCnt;
    int32_t  headsLate;

    uint64_t tailShifter[2];
    uint16_t tailsCnt;
    int32_t  tailsLate;
};

SemaphoreHandle_t radioOwnerMutex;

//------------------------------------------------------------
// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include "boards.h"

uint8_t vertPositionInPixels = 0;
uint8_t horzPositionInPixels = 0;

// no reset on ttgo for lcd
SSD1306Wire oled(0x3c, I2C_SDA, I2C_SCL);
//--------------------------------------------
#include "font.h"
//static const uint8_t *charSet = Roboto_Mono_14; //Nimbus_Mono_L_Regular_16; // Roboto_Mono_48/32/14/_10;
static const uint8_t *charSet = DejaVu_Sans_Mono_10 ; // Roboto_Mono_48/32/14/_10;

#if 0
int oprintf(const char * format,...)
{
  int ret;
  char buffer[120];

  va_list args;
  va_start (args, format);
  uint8_t charHeightPixels = pgm_read_byte(charSet + HEIGHT_POS);
  uint8_t charWidthPixels = pgm_read_byte(charSet + WIDTH_POS);

  // mod true physical to align with a char boundary
  const uint16_t displayWidthInPixels = (oled.width() /charWidthPixels) * charWidthPixels ;
  uint16_t displayHeightInPixels = (oled.height() / charHeightPixels) * charHeightPixels;

  uint16_t maxNumHChars = max(displayWidthInPixels/charWidthPixels, 1);
  //Serial.printf("max no chars = %d\n", maxNumHChars);

  //Serial.printf("width adj=%d  real=%d\n", displayWidthInPixels, oled.width());
  //Serial.printf("width adj=%d  real=%d\n", displayHeightInPixels, oled.height());

  // THIS FUCKING SUCKS
  ret = vsnprintf (buffer,maxNumHChars,format, args);
#if defined JTAG_PRESENT
  Serial.printf("%s: %s\n", __FUNCTION__, buffer);
#endif

  // erase old text
  oled.setColor(BLACK);
  oled.fillRect(0, vertPositionInPixels, displayWidthInPixels, charHeightPixels);

  oled.setColor(WHITE);

  buffer[maxNumHChars] = 0;  // truncate string if too long for font and display

  oled.drawString(0, vertPositionInPixels, buffer);

  //for debug. Write a boarder around the writable text region.
  //oled.drawRect(0, 0, displayWidthInPixels, displayHeightInPixels); // logical phys disp.
  //oled.drawRect(0, 0, oled.width(), oled.height()); // max phys disp.

  oled.display();

  vertPositionInPixels += charHeightPixels;
  if (vertPositionInPixels + charHeightPixels > oled.height()) vertPositionInPixels = 0; //zero based hence >=

  va_end (args);

  return ret;
}
#endif


int oprintf(uint8_t row, const char * format,...)
{
    int ret = 0;
    char buffer[120];

    va_list args;
    va_start (args, format);

    uint8_t charHeightPixels = pgm_read_byte(charSet + HEIGHT_POS);
    uint8_t charWidthPixels = pgm_read_byte(charSet + WIDTH_POS);

    // mod true physical to align with a char boundary
    const uint16_t displayWidthInPixels = (oled.width() /charWidthPixels) * charWidthPixels ;
    uint16_t displayHeightInPixels = (oled.height() / charHeightPixels) * charHeightPixels;

    uint16_t maxNumHChars = max(displayWidthInPixels/charWidthPixels, 1);
    //Serial.printf("max no chars = %d\n", maxNumHChars);

    uint8_t maxRowNum = displayHeightInPixels/charHeightPixels - 1;

    //Serial.printf("width adj=%d  real=%d\n", displayWidthInPixels, oled.width());
    //Serial.printf("width adj=%d  real=%d\n", displayHeightInPixels, oled.height());

    if (row <= maxRowNum)
    {
        vertPositionInPixels = row * charHeightPixels;

        ret = vsnprintf (buffer, sizeof(buffer)-1,format, args);
        //Serial.printf("kkkkkkkkkkk maxNumHChars=%d ret=%d\n", maxNumHChars, ret);

        // erase old text
        oled.setColor(BLACK);
        oled.fillRect(0, vertPositionInPixels, displayWidthInPixels, charHeightPixels);

        oled.setColor(WHITE);

        buffer[min(ret, maxNumHChars-1)] = 0;  // truncate string if too long for font and display

#if defined JTAG_PRESENT
        Serial.printf("%s(%d)(%d vs %d): %s\n", __FUNCTION__, row, ret, maxNumHChars-1, buffer);
#endif
        oled.drawString(0, vertPositionInPixels, buffer);

        //for debug. Write a boarder around the writable text region.
        //oled.drawRect(0, 0, displayWidthInPixels, displayHeightInPixels); // logical phys disp.
        //oled.drawRect(0, 0, oled.width(), oled.height()); // max phys disp.

        oled.display();

        vertPositionInPixels += charHeightPixels;
        if (vertPositionInPixels + charHeightPixels > oled.height()) vertPositionInPixels = 0; //zero based hence >=
    }
    else
    {
      Serial.printf("%s, row out of bounds. Range 0..%d\n", __FUNCTION__, maxRowNum);
      return 0;
    }

    va_end (args);

    return ret;
}

//---------------------------------------
void quickLog(const char * format,...)
{
    va_list args;
    va_start (args, format);

    int ret;
    char buffer[120];
    char output[150];

    time_t epoch;
    time(&epoch);
    struct tm ts_now;

    ts_now = *localtime(&epoch);

    unsigned long now = micros();

    char  tbuffer[20];
    strftime(tbuffer, sizeof(tbuffer), "%m-%d %H:%M", &ts_now);

    char cSummary = 'X';  // general message

    ret = vsnprintf (buffer,sizeof(buffer)-1,format, args);
    sprintf(output, "%c %lu  %s | %s \n", cSummary, epoch, tbuffer, buffer);

    log2SD(output);
    Serial.println(output);

    va_end (args);

}


//---------------------------------------
char one2oneMapping (const unsigned char foo)
{
    return foo; // default UTF-8 mapper kills all chars > 127 :P
}
//---------------------------------------

int setOLED(void)
{
    oled.init();
    oled.clear();
    oled.flipScreenVertically();

    oled.setFontTableLookupFunction(one2oneMapping);  // now chars above 127 will be printed!!!!!!

    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.setFont(charSet);
    oprintf(1,"built on:");
    oprintf(2,__DATE__" "__TIME__);

#if 0
    // test extended character set (nothing b/n 0x80-0x9F)
    for (int i = 0xA0; i < 0x100; i++)
    {
        oprintf("%02X=%c", i, i);
        oled.display();
        delay(500);
    }
#endif

}
//------------------------------------------------------------

volatile int timerCntISR;    // Trigger
volatile int bTimerRunning;

int totalInterrupts;   // counts the number of triggering of the alarm

hw_timer_t * hSilenceTimer = NULL;

portMUX_TYPE criticalSection = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimeOut()
{
   portENTER_CRITICAL_ISR(&criticalSection);
   timerCntISR++;
   bTimerRunning = false;  // needs re-init now.

   IRQ2TaskMessage(true);
   digitalWrite(GREEN_LED, 0);

   portEXIT_CRITICAL_ISR(&criticalSection);
}

//-------------------------------------------------------------

void setupDeadAirTimer()
{

    hSilenceTimer = timerBegin(0, 80, true);  // 80Mhz/80 = 1Mhz tick rate
    timerAttachInterrupt(hSilenceTimer, &onTimeOut, true);

    // Sets an alarm to sound every X mS
    timerAlarmWrite(hSilenceTimer, DEAD_AIR_TIMEmS, false);  // no auto reload

    //timerSetAutoReload(silenceTimer, false);  // one shot?

    // wait til ready.
    timerAlarmEnable(hSilenceTimer);

}

//-------------------------------------------------------------
void irqTask_RSSI( void * parameter )
{
   AMessage rssiMessage;
   Serial.printf("RSSI/DIO task running\n");

   while( DIO0queue != 0 )
   {
      if ( xQueueReceive( DIO0queue, &( rssiMessage ), ( portTickType ) 10000 ) )
      {
        Serial.printf("RSSI items InQ = %4d processing item = %4d pin = %1d dtime = %8d\n",
                    rssiChangeCtr,
                    rssiMessage.changeCnt,
                    rssiMessage.bPinState,
                    rssiMessage.diffTime);
      }
   }
   vTaskDelete(NULL);
}

//-------------------------------------------------------------
uint32_t decodeCounter = 0;

void irqTask_DATA( void * parameter )
{
   AMessage procMessage;
   Serial.printf("DATA task running\n");

   while( DIO2Dataqueue != 0 )
   {
      // if no response in 2mS then the data burst is finished.
      if ( xQueueReceive( DIO2Dataqueue, &( procMessage ),  1000 *portTICK_PERIOD_MS ))
      {

        // just noise.
        if (procMessage.tailShifter[0] == 0 && procMessage.tailShifter[1] == 0) continue;

        uint128DumpInBinary(procMessage.tailShifter,"raw");

        // no need to demanchester, ISR already done it.

        leftJustify128TillOne(&procMessage.tailShifter[0]);
        uint128DumpInBinary(procMessage.tailShifter,"justify");

        ShiftLeft128(&procMessage.tailShifter[0]);  //trim off the sync "5 .. 101" by pushing over the edge
        ShiftLeft128(&procMessage.tailShifter[0]);
        ShiftLeft128(&procMessage.tailShifter[0]);
        uint128DumpInBinary(procMessage.tailShifter,"no-sync");

        revEndian64ByBits (&procMessage.tailShifter[0]);
        revEndian64ByBits (&procMessage.tailShifter[1]);
        uint128DumpInBinary(procMessage.tailShifter,"end4end");

        OREGON_COMMON test;
        test.bits.array[0] = procMessage.tailShifter[0];
        test.bits.array[1] = procMessage.tailShifter[1];

        Serial.print (procMessage.tailShifter[0], HEX); Serial.print("."); Serial.println(procMessage.tailShifter[1], HEX);

        uint32_t device;
        device = test.sensid3 * 0x1000 + test.sensid2 * 0x100 + test.sensid1 * 0x10 + test.sensid0;

        switch (device)
        {
            case _THGN132   : //0x1D20 //Temperature, humidity, 3 channels,
            case _THGN500   : //0x1D30 //Temperature, humidity,
            case _THN132    : //0xEC40 //Temperature, 3 channels,
            case _RTGN318   : //0x0CC3 //Temperature, humidity, time, 5 channels,
            case _RTHN318   : //0x0CD3 //Temperature, time, 5 channels,
            case _RFCLOCK   : //0x0CF3 //
            case _BTHGN129  : //0x5D53 //Temperature, humidity, pressure, 5 channels,
            case _BTHR968   : //0x5D60 //Temperature, humidity, pressure,
            case _THN800    : //0xC844 //Temperature, 3 channels,
            case _UVN800    : //0xD874 //UV index, illuminance (thanks to XOR for the data provided).
                Serial.printf("TODO");
            break;

            case _PCR800    : //0x2914 //precipitation counter
                parsePCR800(device, &test.bits);
                decodeCounter++;
            break;

            case _WGR800    : //0x1984 //Wind direction and speed
                parseWGR800(device, &test.bits);
                decodeCounter++;
            break;
            case _THGR810   : //0xF824 //Temperature, humidity, 10 channels,
                parseTHGR810(device, &test.bits);
                decodeCounter++;
            break;

            default:
                //Serial.printf("unknown device 0x%X\n\n", device); //noise most likely
            break;

        }

        oprintf(3,"%d %3d/%3ddb ", decodeCounter, -ookCalSetting / 2, -rssiCalSetting / 2);

      }
      else
      {
         ////////////////  if (!bCalInProgress) Serial.print('.');
      }

      yield();
   }
   vTaskDelete(NULL);
}

//-------------------------------------------------------------
void irqTask_CLK( void * parameter )
{
   AMessage DO1ClkMessage;
   Serial.printf("CLK task running\n");

   while( DIO1queue != 0 )
   {
      if ( xQueueReceive( DIO1queue, &( DO1ClkMessage ), ( portTickType ) ~0 ) )
      {

        #if 1
        Serial.printf("CLK items InQ = %4d processing item = %4d pin = %1d dtime = %8d\n",
                    clkChangeCtr,
                    DO1ClkMessage.changeCnt,
                    DO1ClkMessage.bPinState,
                    DO1ClkMessage.diffTime);
        #endif

      }
   }
   vTaskDelete(NULL);
}

//-------------------------------------------------------------
typedef struct oneSXentry {
    int8_t address;
    uint8_t data;
    uint8_t lbit;
    uint8_t rbit;
    const char *comment;
};

#define _LNA_GAIN 5   // restrict from 1..6 zero not accomodated

// *** SIGH
// precanned SX172x defines are pre shifted, any others must be manuall shifted !!!!!!!

oneSXentry manySXEntries [ ] =
{
    {SX127X_REG_OP_MODE,SX127X_STANDBY,                                     2, 0, "STANDBY"},

    {SX1278_REG_MODEM_CONFIG_3, SX1278_AGC_AUTO_OFF,                        2, 2, "AGC OFF"},

    {SX127X_REG_LNA,            _LNA_GAIN << 5,                             7, 5, "LNA GAIN"},
    {SX127X_REG_LNA,            SX127X_LNA_BOOST_ON,                        1, 0, "LNA BOOST ON"},

    {SX127X_REG_RX_CONFIG,      SX127X_AGC_AUTO_OFF,                        3, 3, "AGC OFF (again)"},
    {SX127X_REG_RX_CONFIG,      SX127X_AFC_AUTO_OFF,                        4, 4, "AFC OFF"},

    {SX127X_REG_RSSI_CONFIG,    SX127X_RSSI_SMOOTHING_SAMPLES_4,            2, 0, "RSSI 16 samples"},

//    WTF. System breaks if below is enabled ????
//   {SX127X_REG_OOK_PEAK,       SX127X_BIT_SYNC_OFF,                        5, 5, "BitSYNC ON (CLK ON)" },

    {SX127X_REG_OOK_PEAK,       SX127X_OOK_THRESH_FIXED,                    4, 3, "OOK fixed threshold" },
    {SX127X_REG_OOK_AVG,        SX127X_OOK_PEAK_THRESH_DEC_16_1_CHIP,       7, 5, "Slowest OOK fall rate" },

    {SX127X_REG_PREAMBLE_DETECT, SX127X_PREAMBLE_DETECTOR_OFF,              7, 7, "PREAMBLE DET ON" },
    {SX127X_REG_PREAMBLE_DETECT, SX127X_PREAMBLE_DETECTOR_2_BYTE,           6, 5, "PREAMBLE size 2 bytes" },
    {SX127X_REG_PREAMBLE_DETECT, SX127X_PREAMBLE_DETECTOR_TOL,              4, 0, "PREAMBLE error 'TOL' tolerance" },

    {SX127X_REG_SYNC_CONFIG,     SX127X_SYNC_OFF,                           4, 4, "SYNC det is OFF (1 is ON - default)"},
    {SX127X_REG_SYNC_CONFIG,     SX127X_PREAMBLE_POLARITY_55,               5, 5, "SYNC is 55 ( 0 is AA)"},
    {SX127X_REG_SYNC_CONFIG,     0,                                         2, 0, "SYNC size is val+1 bytes"},

    {SX127X_REG_PACKET_CONFIG_1, SX127X_DC_FREE_MANCHESTER,                 6, 5, "DC - no Manchester (raw)"},
    {SX127X_REG_PACKET_CONFIG_1, SX127X_ADDRESS_FILTERING_OFF,              2, 1, "No address"},

    {SX127X_REG_DIO_MAPPING_1,   SX127X_DIO0_CONT_RSSI_PREAMBLE_DETECTED,   7, 6, "DIO0 = RSSI/Preamble"},

    {SX127X_REG_DIO_MAPPING_1,  SX127X_DIO1_CONT_DCLK,                      5, 4, "DCLK ON"},
    {SX127X_REG_DIO_MAPPING_1,  SX127X_DIO2_CONT_DATA,                      3, 2, "DATA ON"},
    {SX127X_REG_PACKET_CONFIG_2,SX127X_DATA_MODE_CONTINUOUS,                6, 6, "Continuous - no packet"},

    {SX127X_REG_OP_MODE,        SX127X_MODULATION_OOK,                      6, 5, "RECIEVE OOK format"},
    {SX127X_REG_OP_MODE,        SX127X_RX,                                  2, 0, "RECIEVE enabled"},
    {SX127X_REG_RX_CONFIG,      SX127X_AGC_AUTO_ON,                         3, 3, "LNA gain controlled by AGC (=1)"},

    { -1, 0, 0, 0 }
};

int16_t manualProgram (void)
{
    int i = 0;
    int16_t status = 0;

    while ( manySXEntries[i].address >= 0 )
    {
        Serial.printf("[0x%02X] set to 0x%02X in bits %d-%d\t%s\n",
                        manySXEntries[i].address,
                        manySXEntries[i].data,
                        manySXEntries[i].lbit,
                        manySXEntries[i].rbit,
                        manySXEntries[i].comment);

        status |= radio.lowlevel(manySXEntries[i].address,
                                 manySXEntries[i].data,
                                 manySXEntries[i].lbit,
                                 manySXEntries[i].rbit);

        i++;
    }
    Serial.printf("%s status = %d\n", __FUNCTION__, status);
    return status;
}

int setRadio(float freq, float bw)
{

    //while (xSemaphoreTake( radioOwnerMutex, ( TickType_t ) 10 ) != pdTRUE );

    radio.reset();
    int state = radio.beginFSK(/*float freq =     */ freq,
                               /*float br =       */ min(bw/2.0,32.0), //1.2,
                               /*float freqDev =  */ 50.0,
                               /*float rxBw =     */ bw,
                               /*int8_t power =   */ 10,
                               /*uint16_t preambleLength =    */16,
                               /*bool enableOOK = */ true);

    if (state == ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true);
    }

    pinMode(DIO0, INPUT);
    pinMode(DIO1, INPUT);
    pinMode(DIO2, INPUT);

#if HARD_OREGON
    Serial.println("NOT USING SX1276, OREGON HARD ON CHIP");
    delay(2000);

    attachInterrupt(DIO2, OnDIO2DataISR, CHANGE);
#else
    radio.setDio2Action(ISR_DIO2Data);
    radio.setDio0Action(ISR_DIO0Preamble);
//  radio.setDio1Action(OnDIO1ClkISR);
#endif

    radio.setGain(1);

    manualProgram();

    radio.setOokFixedOrFloorThreshold(ookCalSetting);  // restore ook, cleared on set freq.
    radio.lowlevel(SX127X_REG_OOK_PEAK, 1 << 3, 4, 3);


    //radio.receiveDirect();
    radio.receiveBitDirect();

    //radio.setSyncWord(syncWord, sizeof(syncWord));

    Serial.printf("******* frequency = %f Mhz  bw = %f Khz\n", freq, bw); delay(1000);
    //xSemaphoreGive(radioOwnerMutex);

    return state;
}
//-------------------------------------------------------------
#define OREGON_YELLOW  433.9036
#define MID_FREQ       433.92
#define OREGON_TRAILER 433.839800

#define RX_ISM_LO   433.05
#define RX_ISM_HI   434.79
#define RX_ISM_MID  (RX_ISM_HI+RX_ISM_LO)/2


#define BANDWIDTH_KHZ_NORM    249

// cal frequency is 2 times the bw away from the highest ISM freq.
#define FREQ_MHZ_NORM     RX_ISM_MID

#define FREQ_MHZ_CAL_HI      ((RX_ISM_HI * 1000000. + 2.0 * BANDWIDTH_KHZ_NORM * 1000.)/ 1000000.)
#define FREQ_MHZ_CAL_LO      ((RX_ISM_LO * 1000000. - 2.0 * BANDWIDTH_KHZ_NORM * 1000.)/ 1000000.)


//-------------------------------------------------------------
#define SPIN 100

void ScopeBlips(int numBlips)
{
    static volatile uint64_t spin; //keep out of optimization, use voilatile
    while (numBlips--)
    {

        spin = SPIN;
        digitalWrite(SCOPE_GPIO35, 1);
        while (spin--);
        spin = SPIN;
        digitalWrite(SCOPE_GPIO35, 0);
        while (spin--);
    }
}
//-------------------------------------------------------------------------
void setFrequency(float freq)
{
      radio.setFrequency(freq);
      Serial.printf("\n%s to %f\n", __FUNCTION__, freq);
      delay(100); //settle pll

      // no idea why this gets cleared. Restore it.
      radio.setOokFixedOrFloorThreshold(ookCalSetting);  // restore ook, cleared on set freq.
      radio.lowlevel(SX127X_REG_OOK_PEAK, 1 << 3, 4, 3);

      radio.receiveBitDirect();
}
//-------------------------------------------------------------------------
void RssiCalibrateThread(void *notUsed)
{
    Serial.printf("%s active\n", __FUNCTION__);
    while (true)
    {
        uint16_t hi, lo;
        while (xSemaphoreTake( radioOwnerMutex, ( TickType_t ) 1000 ) != pdTRUE );
        bCalInProgress = true;

        Serial.printf("%s start ----------\n", __FUNCTION__);

        setFrequency(FREQ_MHZ_CAL_HI);
        hi = radio.calRSSI();
        quickLog("%s setting hi    = 0x%02X (%4.1f db)", __FUNCTION__, hi, -(float)hi / 2.0);


        setFrequency(FREQ_MHZ_CAL_LO);
        lo = radio.calRSSI();
        quickLog("%s setting lo    = 0x%02X (%4.1f db)", __FUNCTION__, lo, -(float)lo / 2.0);

        rssiCalSetting = max( lo, hi);
        radio.calRSSI(rssiCalSetting);
        quickLog("%s setting final = 0x%02X (%4.1f db)", __FUNCTION__, rssiCalSetting, -(float)rssiCalSetting / 2.0);

        setFrequency(FREQ_MHZ_NORM);
        //setRadio(FREQ_MHZ_NORM, BANDWIDTH_KHZ_NORM);  // setup freq AND BANDWIDTH

        Serial.printf("%s sleep ----------\n", __FUNCTION__);

        bCalInProgress = false;

        xSemaphoreGive(radioOwnerMutex);
        delay(1000 * 60 * 30);  // rssi cal every 1/2 hr.

    }
}
//-------------------------------------------------------------------------

void OokCalibrateThread(void *notUsed)
{
    Serial.printf("%s active\n", __FUNCTION__);
    while(true)
    {
        uint16_t hi, lo;
        while (xSemaphoreTake( radioOwnerMutex, ( TickType_t ) 1000 ) != pdTRUE );
        bCalInProgress = true;
        Serial.printf("%s start ----------\n", __FUNCTION__);

        setFrequency(FREQ_MHZ_CAL_HI);
        hi = radio.calOOK();   // find out what it is
        quickLog("%s setting hi end = 0x%02X (%4.1f db)", __FUNCTION__, hi, -(float)hi / 2.0);

        setFrequency(FREQ_MHZ_CAL_LO);
        lo = radio.calOOK();   // find out what it is
        quickLog("%s setting lo end = 0x%02X (%4.1f db)", __FUNCTION__, lo, -(float)lo / 2.0);

        ookCalSetting = max (hi, lo);

        setFrequency(FREQ_MHZ_NORM);
        radio.calOOK(ookCalSetting);    // set and forget (must occur after set freq);

        quickLog("%s setting final  = 0x%02X (%4.1f db)", __FUNCTION__, ookCalSetting, -(float)ookCalSetting / 2.0);

        Serial.printf("%s sleep ----------\n", __FUNCTION__);

        bCalInProgress = false;
        xSemaphoreGive(radioOwnerMutex);
        delay(1000 * 60 * 60 * 4 );  // call every 4 hours.
    }
}
//-------------------------------------------------------------------------
void print_reset_reason(/*RESET_REASON*/ int reason)
{
  Serial.printf("reset value = %d ", reason);
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3, Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

//-------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(1000);
  setOLED();

  // auto detect if a DUMB 433 is on the esp32
  Serial.printf("checking pin %d for dumb 433 device\n", DUMB_433_INPUT);

  pinMode(DUMB_433_INPUT, INPUT);

  uint64_t spin = micros() + 2000000;
  bool look = digitalRead(DUMB_433_INPUT);
  uint32_t bFoundDumb433 = 0;

  while (spin > micros())
  {
    if (look != digitalRead(DUMB_433_INPUT))
    {
        bFoundDumb433++; look = !look;
    }
  }
  Serial.printf("hit ctr %d\n", bFoundDumb433);


  Serial.println("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));

  int cpu0Reset = rtc_get_reset_reason(0);
  Serial.println("CPU0 reset reason: ");
  print_reset_reason(cpu0Reset);

  int cpu1Reset = rtc_get_reset_reason(1);
  Serial.println("CPU1 reset reason: ");
  print_reset_reason(cpu1Reset);

  pinMode(GREEN_LED,OUTPUT);
  ScopeBlips(10);

  delay(1000);
  Serial.println(bFoundDumb433 ? BOOT_MSG1 : BOOT_MSG2);

  setupWiFi();  // setup the time first.

#if !JTAG_PRESENT
  setupFS();    // fs needs time from wifi.

  //----------------------------------------------------------

  // take a quick look and see if the scope pin is grounded.
  // if it is grounded, that means erase the SD CARD.

  pinMode(SCOPE_GPIO35,INPUT_PULLUP);
  bool scope = digitalRead(SCOPE_GPIO35);
  Serial.printf("checking GPIO%d = %d, %s erasing SD card\n", SCOPE_GPIO35, scope, scope ? "NOT":"");

  if (cpu1Reset != 14 || ERASE_SD_ONPOWERUP || !digitalRead(SCOPE_GPIO35))
  {
    eraseOregon();
    Serial.printf("erase done. Waiting for GPIO%d to be released\n", SCOPE_GPIO35);
    while (!digitalRead(SCOPE_GPIO35)) delay(500);
  }

#endif

  dumpGuinessRecords();


  // now SCOPE_GPIO is an output for the scope.
  pinMode(SCOPE_GPIO35,OUTPUT);

  radioOwnerMutex = xSemaphoreCreateMutex();
  assert(radioOwnerMutex);

  setupDeadAirTimer();

  if (bFoundDumb433)
  {
      attachInterrupt(DUMB_433_INPUT,ISR_DIO2Data, CHANGE);
  }
  else
  {
      // initialize SX1278 with default settings
      Serial.print(F("[SX1278] Initializing ... "));

      setRadio(FREQ_MHZ_CAL_HI, BANDWIDTH_KHZ_NORM);  // setup freq AND BANDWIDTH

      xTaskCreatePinnedToCore(
                      RssiCalibrateThread, /* Function to implement the task */
                      "RssiCalibrateThread",/* Name of the task */
                      10000,           /* Stack size in words */
                      (void *)99,     /* Task input parameter */
                      0,              /* Priority of the task */
                      NULL,           /* Task handle. */
                      1);             /* Core where the task should run */

      xTaskCreatePinnedToCore(
                      OokCalibrateThread,  /* Function to implement the task */
                      "OokCalibrateThread", /* Name of the task */
                      10000,           /* Stack size in words */
                      (void *)99,     /* Task input parameter */
                      0,              /* Priority of the task */
                      NULL,           /* Task handle. */
                      1);             /* Core where the task should run */


      yield();
      Serial.printf("OOK moved to peak mode\n");
   }

#if 0
    RSSI_DIO0queue = xQueueCreate(20, sizeof(AMessage));
    xTaskCreatePinnedToCore(
                    RSSI_task,       /* Function to implement the task */
                    "RssiTask",      /* Name of the task */
                    10000,           /* Stack size in words */
                    (void *)99,      /* Task input parameter */
                    0,               /* Priority of the task */
                    NULL,            /* Task handle. */
                    1);              /* Core where the task should run */

    CLK_DIO1queue  = xQueueCreate(20, sizeof(AMessage));
    xTaskCreatePinnedToCore(
                  CLK_task,       /* Function to implement the task */
                  "DATATask",      /* Name of the task */
                  10000,           /* Stack size in words */
                  (void *)99,      /* Task input parameter */
                  0,               /* Priority of the task */
                  NULL,            /* Task handle. */
                  1);              /* Core where the task should run */
#endif


  DIO2Dataqueue = xQueueCreate(200, sizeof(AMessage));
  xTaskCreatePinnedToCore(
                  irqTask_DATA,      /* Function to implement the task */
                  "DIO2Task",     /* Name of the task */
                  10000,          /* Stack size in words */
                  (void *)99,      /* Task input parameter */
                  0,              /* Priority of the task */
                  NULL,           /* Task handle. */
                  1);             /* Core where the task should run */

}

//-------------------------------------------------------------

void ISR_DIO0Preamble(void)
{
    AMessage RssiMessage;
    bool lastState;
    static unsigned long lastIrqTime;

    unsigned long now = micros();

    lastState = digitalRead(DIO0);

    RssiMessage.absTime = now;
    RssiMessage.changeCnt = rssiChangeCtr++;

    RssiMessage.bPinState = lastState;

    RssiMessage.diffTime = now - lastIrqTime;
    lastIrqTime = now;

    if (DIO0queue) xQueueSendToBackFromISR(DIO0queue, &RssiMessage, NULL);
}

//-------------------------------------------------------------

void ISR_DIO1Clk(void)
{
    AMessage clkMessage;
    bool lastState;
    static unsigned long lastIrqTime;

    unsigned long now = micros();

    lastState = digitalRead(DIO1);

    clkMessage.absTime = now;
    clkMessage.changeCnt = clkChangeCtr++;

    clkMessage.bPinState = lastState;

    clkMessage.diffTime = now - lastIrqTime;
    lastIrqTime = now;

    if (DIO1queue) xQueueSendToBackFromISR(DIO1queue, &clkMessage, NULL);
}

//-------------------------------------------------------------
#define BIT_TIME        (480)
#define SYMBOL_TIME     (BIT_TIME * 2)
#define SYMBOL_ERROR    300

static COIN_TOSS rising = {0};
static COIN_TOSS falling = {0};

static bool bBothInited;
static bool bSyncBitsComplete = true;

//---------------------------------------

void ResetAll(void)
{
    memset(&rising, 0, sizeof(rising));
    memset(&falling, 0, sizeof(falling));
    bBothInited = false;
    bSyncBitsComplete = false;
}

//---------------------------------------

void print64(uint64_t val)
{
    for (int i = 0; i < 64; i++)
    {
        if (i && !(i % 4)) Serial.print('.');
        Serial.print(val & HI_BIT64 ? '1':'0');
        val *= 2;
    }
}
void print64ln(uint64_t val)
{
    print64(val); Serial.println();
}
//---------------------------------------

void RotateShiftLeft(COIN_TOSS *dest, bool bitIn)
{
    if (!bSyncBitsComplete) return;
    dest->bitCnt++;
    dest->shifter[0] *= 2;
    dest->shifter[0] |= !!(dest->shifter[1]  &  HI_BIT64 );
    dest->shifter[1] *= 2;
    dest->shifter[1] |= bitIn;
}
//---------------------------------------
void IRQ2TaskMessage(bool bForced)
{
    // both sides tapped out?
    if ( (rising.arrivalTime < 0 && falling.arrivalTime < 0) || bForced)
    {
        // only if meaningful capture do we send it
        if (rising.bitCnt > 50 || falling.bitCnt > 50)
        {
            AMessage NewRxMessage;
            NewRxMessage.headShifter[0] = rising.shifter[0];
            NewRxMessage.headShifter[1] = rising.shifter[1];
            NewRxMessage.headsCnt  = rising.bitCnt;
            NewRxMessage.headsLate = rising.lateTime;

            NewRxMessage.tailShifter[0] = falling.shifter[0];
            NewRxMessage.tailShifter[1] = falling.shifter[1];
            NewRxMessage.tailsCnt  = falling.bitCnt;
            NewRxMessage.tailsLate = falling.lateTime;
            NewRxMessage.debug = HI_BIT64;

            // both sides have tapped out.
            if (DIO2Dataqueue) xQueueSendToBackFromISR(DIO2Dataqueue, &NewRxMessage, NULL);
        }
        ResetAll();
    }
}
//---------------------------------------

void ISR_DIO2Data(void)
{
    static bool bTossHeads;
    static uint64_t lastIrqTime;
    bool   lastState;
    uint64_t long diffTime;
    static bool test;


    // our hardware solution has a ook squelch. :)
    // Ironically a perfectly formed packet will not generate a transmission until
    // the NEXT edge occurs (which could be 60 seconds later)  :(
    // So to make the last packet get processed quickly, wait for silence, then send.

    if (!bTimerRunning)
    {
        bTimerRunning = true;
        // Sets an alarm to sound every X mS
        timerAlarmWrite(hSilenceTimer, DEAD_AIR_TIMEmS, false);  // no auto reload
        timerAlarmEnable(hSilenceTimer);
        digitalWrite(GREEN_LED,1);
    }


    timerWrite(hSilenceTimer, 0); //retriggerable - one shot (reset silenceTimer)

    if (bCalInProgress) return;   // radio cal in progress.

    signed long now = micros(); // keep it signed.

    lastState = !digitalRead(DIO2);
    diffTime = now - lastIrqTime;

    if (bBothInited == false)
    {

        if (!rising.arrivalTime && lastState == 0)
        {
            rising.arrivalTime  = now + SYMBOL_TIME;
            rising.now = now;
            rising.shifter[0] = 0;
            rising.shifter[1] = 0;
            rising.bitCnt  = 0;

            if (falling.now > 0)            // other side inited ?
            {
                if ( (now - falling.now) < 400)
                {
                    ResetAll();
                    return;
                }
                bBothInited = true;
            }
        }

        if (!falling.arrivalTime && lastState == 1)
        {
            falling.arrivalTime  = now + SYMBOL_TIME;
            falling.now = now;
            falling.shifter[0] = 0;
            falling.shifter[1] = 0;
            falling.bitCnt = 0;

            if (rising.now > 0)    // other side inited ?
            {
                if ( (now - rising.now) < 400)
                {
                    ResetAll();
                    return;
                }
                bBothInited = true;
            }
        }

        return;        // both sides are intialized if you got here.
    }

    //--------------------------------------------

    //if (now > rising.arrivalTime - ERROR_WIDTH)  // not a glitch
    if ( now >  rising.arrivalTime - SYMBOL_TIME /4)  // noise gate.
    {
        if ( now <  rising.arrivalTime + SYMBOL_ERROR  &&
             now >  rising.arrivalTime - SYMBOL_ERROR  &&
             rising.arrivalTime > 0)
        {
            //within window
            RotateShiftLeft(&rising, lastState);

            rising.arrivalTime  = now + SYMBOL_TIME;
            rising.now = now;
            //blip(2);
        }
        else
        {
            if (rising.arrivalTime > 0)
            {
                rising.lateTime = now - rising.arrivalTime;
                rising.arrivalTime  = -1;
                bSyncBitsComplete = true;

                // all sync past, preamble just starting
                //blip(2);
            }
        }
    }


    //if (now > falling.arrivalTime - ERROR_WIDTH) // not a glitch

    if ( now >  falling.arrivalTime - SYMBOL_TIME /4)  // noise gate.
    {
        if (now <  falling.arrivalTime + SYMBOL_ERROR  &&
            now >  falling.arrivalTime - SYMBOL_ERROR )
        {
            //within window
            RotateShiftLeft(&falling, lastState);

            falling.arrivalTime  = now + SYMBOL_TIME;
            falling.now = now;

            // blip for each clock edge
            if (bSyncBitsComplete) // comment out if you want to see sync phase.
            {
                ScopeBlips(1);
            }
        }
        else
        {
            if (falling.arrivalTime > 0)
            {
                falling.lateTime = now - falling.arrivalTime;
                falling.arrivalTime  = -1;
                bSyncBitsComplete = true;
                //blip(3);
            }
        }
     }

    /////////////////////     IRQ2TaskMessage(false);
}

void loop()
{
   // only needed for watchdog.
   vTaskSuspend(NULL);
}

