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

        uint64_t rateLo:4;
        uint64_t rateMd:4;
        uint64_t rateHi:4;

        uint64_t totalLo:4;
        uint64_t totalMd:4;
        uint64_t totalHi:4;

        uint64_t _unkLo:4;
        uint64_t _unkMd:4;

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
     uint64x2_t bits;
    };
}PCR800;

void parsePCR800(uint16_t device, uint64x2_t *input)
{
	PCR800 *stuff = (PCR800 *) input;
	guinessRecord * aRecord;
	bool bRepeat = false;

	if (!check_oregon_crcsum((OREGON_COMMON*) input, 0X07, 0X00, 22, true)) return;


	uint8_t channel = stuff->channel;
	float rateMMhr = (float)(stuff->rateHi * 100 + stuff->rateMd * 10 + stuff->rateLo) / 10.;
	float total = (float)(stuff->totalHi * 100 + stuff->totalMd * 10 + stuff->totalLo) / 10.;

	Serial.printf("%s channel : %d\n", __FUNCTION__, channel);
	Serial.printf("%s rate    : %+.1f mm/hr\n", __FUNCTION__, rateMMhr);
	Serial.printf("%s total   : %+.1f mm\n", __FUNCTION__, total);

	aRecord = locateDeviceStats(device, channel);

	time_t epoch;
	time(&epoch);

	char cStatus = 'C'; // assume change occured

	if (aRecord->outsideRainRate == rateMMhr)
	{
		bRepeat = true;
		cStatus = ':';  // no change occured.
	}

	unsigned long now = micros();

	aRecord->outsideRainRate= rateMMhr;

    char rb = '|';
	if (rateMMhr > aRecord->outsideRainRate)
	{
		aRecord->outsideRainRate = rateMMhr;
		aRecord->outsideRainRateTime = epoch;
		cStatus = 'R';	// a record occured.
		rb = '*';
	}

	char tb = '|';
	if (total > aRecord->outsideRainTotal)
	{
		aRecord->outsideRainTotal = total;
		aRecord->outsideRainTotalTime = epoch;
		cStatus = 'R'; // a record occured.
		tb = '*';
	}
	if (aRecord->lastArrivalTime) aRecord->deltaArrivalTime = now - aRecord->lastArrivalTime;
	aRecord->lastArrivalTime = now;

	struct tm	ts_total;
	struct tm	ts_rate;
	struct tm	ts_now;

	char  rateBuf[20];
	char  totalBuf[20];
	char  nowbuf[20];

	oprintf(2, "rain %.1f mm/s %.1f mm", aRecord->outsideRainRate, aRecord->outsideRainTotal);

	ts_rate = *localtime(&aRecord->outsideRainRateTime);
	ts_total = *localtime(&aRecord->outsideRainTotalTime);
	ts_now = *localtime(&epoch);

	strftime(rateBuf,  sizeof(rateBuf), "%m-%d %H:%M", &ts_rate);
	strftime(totalBuf,  sizeof(totalBuf), "%m-%d %H:%M", &ts_total);
	strftime(nowbuf, sizeof(nowbuf), "%m-%d %H:%M", &ts_now);

	char output[150];
	sprintf(output, "%c dev=0x%04X ch=%02d %lu  %s %+05.1f mm/s %c pk %s %+05.1f mm/S %c tot %s %+05.1f mm   | %lu mS \n",
		cStatus,
		device, channel,
	    epoch,
	    nowbuf, rateMMhr,
		rb, rateBuf, aRecord->outsideRainRate,
		tb,totalBuf, aRecord->outsideRainTotal,
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

