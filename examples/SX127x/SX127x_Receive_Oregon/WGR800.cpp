#include "TypeDef.h"
#include "Module.h"
#include "OregonV2.h"

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

        uint64_t compass:4;
        uint64_t resv1:4;
        uint64_t resv2:4;
        uint64_t speedLo:4;

        uint64_t speedMd:4;
        uint64_t speedHi:4;
        uint64_t aspdLo:4;
        uint64_t aspdMd:4;

		uint64_t aspdHi:4;
        uint64_t sumLo:4;
        uint64_t sumHi:4;

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
     uint64x2_t  uint64x2;
    };
}WGR800;

void parseWGR800(uint16_t device, uint64x2_t *input)
{

	WGR800 *stuff = (WGR800 *) input;
	guinessRecord * aRecord;
	bool bRepeat = false;


#if 0
	uint32_t xdevice;
	xdevice = stuff->sensid3 * 0x1000 + stuff->sensid2 * 0x100 + stuff->sensid1 * 0x10 + stuff->sensid0;
	Serial.printf("%s: xdevice = 0x%X\n", __FUNCTION__, xdevice);

	//Serial.printf("%s: 0x%0lX 0x%0lX \n", __FUNCTION__, stuff->bits.array[0], stuff->bits.array[1]);
	Serial.print(stuff->uint64x2.array[0], HEX); Serial.print(" "); Serial.println(stuff->uint64x2.array[1], HEX);

	for (int i = 0; i < 16; i++)
	{
		Serial.printf("[%d] = 0x%02X  logic [MAP=%d] = 0x%02X  \n", i, stuff->uchar8x16.array[i], LOGICAL2PHYSICAL(i), stuff->uchar8x16.array[LOGICAL2PHYSICAL(i)]);
	}
#endif

	if(!check_oregon_crcsum((OREGON_COMMON *) input, 0X07, 0X00, 21, true)) return ;  //WGR

	uint8_t channel = stuff->channel;
	float peak    = (float)(stuff->speedHi * 100 + stuff->speedMd * 10 + stuff->speedLo) / 10.;
	float average = (float)(stuff->aspdHi  * 100 + stuff->aspdMd  * 10 + stuff->aspdLo)  / 10.;

	peak = peak * 3.6;  // m/s -> kph
	average = average * 3.6; // m/s -> kph

	float_t direction  = (float) (stuff->compass * 225) / 10.;

	Serial.printf("%s channel : %d\n", __FUNCTION__, channel);
	Serial.printf("%s dir     : %+.1f\n", __FUNCTION__, direction);
	Serial.printf("%s peak   : %+.1f\n", __FUNCTION__, peak);
	Serial.printf("%s average : %+.1f\n", __FUNCTION__, average);

	aRecord = locateDeviceStats(device, channel);
	time_t epoch;
	time(&epoch);
	char cSummary = 'C';  // assume a change occured

	unsigned long now = micros();

	if (average == aRecord->outsideWindAvgHi && peak == aRecord->outsideWindPeakHi)
	{
		bRepeat = true;
		cSummary= ':';	// no change occured
	}

	aRecord->outsideWindNow = average;
	aRecord->outsideWindCompass = direction;
	oprintf(0, "wind %+.1f %.1f kph", average, aRecord->outsideWindPeakHi);

    char cAv='|';
	if (average > aRecord->outsideWindAvgHi)
	{
		aRecord->outsideWindAvgHi = average;
		aRecord->outsideWindAvgTime = epoch;
		cSummary = 'R';  // a record occured
		cAv='*';
	}

    char cPk='|';
	if (peak > aRecord->outsideWindPeakHi)
	{
		aRecord->outsideWindPeakHi = peak;
		aRecord->outsideWindPeakTime= epoch;
		cSummary = 'R';  // a record occured
		cPk = '*';
	}

	if (aRecord->lastArrivalTime) aRecord->deltaArrivalTime = now - aRecord->lastArrivalTime;
	aRecord->lastArrivalTime = now;

	struct tm	ts_peak;
	struct tm	ts_avg;
	struct tm	ts_now;

	char  avgbuf[20];
	char  peakbuf[20];
	char  nowbuf[20];

	ts_avg = *localtime(&aRecord->outsideWindAvgTime);
	ts_peak = *localtime(&aRecord->outsideWindPeakTime);

	ts_now = *localtime(&epoch);

	strftime(avgbuf,  sizeof(avgbuf), "%m-%d %H:%M", &ts_avg);
	strftime(peakbuf,  sizeof(peakbuf), "%m-%d %H:%M", &ts_peak);
	strftime(nowbuf, sizeof(nowbuf), "%m-%d %H:%M", &ts_now);

	char output[150];

	sprintf(output, "%c dev=0x%04X ch=%02d %lu  %s %+05.1f kph  %c av %s %+05.1f kph  %c pk  %s %+05.1f kph  | dir %4.1f | %lu mS \n",
		cSummary,
		device, channel,
	    epoch, nowbuf, peak, // the peak over this sample time
		cAv, avgbuf, aRecord->outsideWindAvgHi,
		cPk, peakbuf, aRecord->outsideWindPeakHi,
		aRecord->outsideWindCompass,
		aRecord->deltaArrivalTime/1000);

	if(!bPrintIfChangeFound) Serial.println(output);

	if (!bRepeat)
	{
		if(bPrintIfChangeFound) Serial.println(output);
		log2SD(output);
    }
	else
 		Serial.printf("no change since last report\n");

	Serial.printf("----------------------\n");
}

