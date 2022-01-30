#ifndef __OREGONV2__
#define __OREGONV2__

#include "TypeDef.h"
#include "Module.h"

#define FULL_DEBUG 0
#define HI_BIT64 ((uint64_t) 1 << ((sizeof(uint64_t)) * 8)-1)


//#define SDCARD_LOGGING    #set to 1 or 0

#ifndef SDCARD_LOGGING
#error "STOP must define above if you want SDCARD logging"
#endif

#if (SDCARD_LOGGING==0)
    #define JTAG_PRESENT 1
#else
    #define JTAG_PRESENT 0
#endif


void dumpGuinessRecords();
void eraseOregon(void); // clear SD files
int log2SD ( const char * format, ... );
void setupWiFi();
void setupFS();
int oprintf(const char * format,...);
int oprintf(uint8_t row, const char * format,...);


#define LINE Serial.printf("%s:%d \n", __FUNCTION__, __LINE__);


//Recognize packets from the following Oregon Scientific sensors:
//
#define _THGN132   0x1D20 //Temperature, humidity, 3 channels,
#define _THGN500   0x1D30 //Temperature, humidity,
#define _THN132    0xEC40 //Temperature, 3 channels,
#define _RTGN318   0x0CC3 //Temperature, humidity, time, 5 channels,
#define _RTHN318   0x0CD3 //Temperature, time, 5 channels,
#define _RFCLOCK   0x0CF3 //
#define _BTHGN129  0x5D53 //Temperature, humidity, pressure, 5 channels,
#define _BTHR968   0x5D60 //Temperature, humidity, pressure,
#define _THGR810   0xF824 //Temperature, humidity, 10 channels,
#define _THN800    0xC844 //Temperature, 3 channels,
#define _WGR800    0x1984 //Wind direction and speed
#define _UVN800    0xD874 //UV index, illuminance (thanks to XOR for the data provided).
#define _PCR800    0x2914 //precipitation counter

typedef struct {
    uint64_t array[2];
} uint64x2_t;

typedef struct {
    unsigned char array[16];  // 16 * 8 = 128 bits.
} uchar8x16_t;

// given logical array index, what is the phyisical array address we need.
//-------------------------------------------------------------
typedef struct {
  union {

    struct {
        uint64_t sensid3:4;
        uint64_t sensid2:4;
        uint64_t sensid1:4;
        uint64_t sensid0:4;

        uint64_t channel:4;
        uint64_t rollhi:4;
        uint64_t rollLo:4;
        uint64_t flags:4;

        uint64_t n8:4;
        uint64_t n9:4;
        uint64_t na:4;
        uint64_t nb:4;

        uint64_t nc:4;
        uint64_t nd:4;
        uint64_t ne:4;
        uint64_t nf:4;



        uint64_t l0:4;
        uint64_t l1:4;
        uint64_t l2:4;
        uint64_t l3:4;

        uint64_t l4:4;
        uint64_t l5:4;
        uint64_t l6:4;
        uint64_t l7:4;

        uint64_t l8:4;
        uint64_t l9:4;
        uint64_t la:4;
        uint64_t lb:4;

        uint64_t lc:4;
        uint64_t ld:4;
        uint64_t le:4;
        uint64_t lf:4;
     };

     uchar8x16_t uchar8x16;
     uint64x2_t bits;
    };
}OREGON_COMMON;

//---------------------------------------
typedef struct guinessRecords_t {
    guinessRecords_t *next;
    uint32_t devChannel;
    float outsideHumidNow;
    float outsideTempNow;
    float outsideTempLo;
    float outsideTempHi;
    float outsideHumidLo;
    float outsideHumidHi;
    time_t outsideTempLoTime;
    time_t outsideTempHiTime;
    time_t outsideHumidHiTime;
    time_t outsideHumidLoTime;

    float outsideWindNow;
    time_t outsideWindTime;

    float outsideWindAvgHi;
    time_t outsideWindAvgTime;
    float outsideWindPeakHi;
    time_t outsideWindPeakTime;
    float outsideWindCompass;

    float outsideRainTotal;
    time_t outsideRainTotalTime;
    float outsideRainRate;
    time_t outsideRainRateTime;

    unsigned long lastArrivalTime;
    unsigned long deltaArrivalTime;

} guinessRecord;

void parseTHGR810(uint16_t device, uint64x2_t *);
void parseWGR800(uint16_t device, uint64x2_t *);
void parsePCR800(uint16_t device, uint64x2_t *);

guinessRecord * locateDeviceStats(uint16_t device, uint8_t channel);

bool check_oregon_crcsum(OREGON_COMMON *oregon_data, byte CCIT_POLY, byte CCIT_START, byte p_length, bool v3);

void ShiftLeft128(uint64_t dest[]);
void leftJustify128TillOne(uint64_t dest[]);
void uint128DumpInBinary(uint64_t dest[], char *msg);
void uint128DumpInHex(uint64_t dest[], char *msg);
uint64_t revEndian64ByBits (uint64_t *what);
void revStringNibbles(uint8_t *what, unsigned int numBits);
extern bool bPrintIfChangeFound;




#endif // __OREGONV2__

