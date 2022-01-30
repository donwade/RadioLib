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

        uint64_t celcLo:4;
        uint64_t celcMd:4;
        uint64_t celcHi:4;
        uint64_t celcSn:4;


        uint64_t humdLo:4;
        uint64_t humdHi:4;
        uint64_t _unk2:4;
        uint64_t _unk1:4;

        uint64_t sumLo:4;
        uint64_t sumHi:4;
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
}THGR810;

void parseTHGR810(uint16_t device, uint64x2_t *input)
{
	THGR810 *stuff = (THGR810 *) input;
	guinessRecord * aRecord;
	bool bRepeat = false;

	if(!check_oregon_crcsum((OREGON_COMMON*) input, 0X07, 0X00, 19, true)) return; // for THGR numbers.

	uint8_t channel = stuff->channel;
	uint8_t humidity = stuff->humdHi * 10 + stuff->humdLo;
	float_t temp = (float) (stuff->celcHi * 100 + stuff->celcMd * 10 + stuff->celcLo) / 10.;
	if (stuff->celcSn) temp = -temp;

	Serial.printf("%s channel : %d\n", __FUNCTION__, channel);
	Serial.printf("%s humidity: %d\n", __FUNCTION__, humidity);
	Serial.printf("%s temp    : %+.1f\n", __FUNCTION__, temp);


	aRecord = locateDeviceStats(device, channel);
	time_t epoch;
	time(&epoch);
	char cStatus = 'C';	// assume change occured.

	unsigned long now = micros();

	if (aRecord->outsideTempNow == temp && aRecord->outsideHumidNow == humidity)
	{
		bRepeat = true;
		cStatus = ':'; // change did not occur
	}

	aRecord->outsideTempNow = temp;
	aRecord->outsideHumidNow = humidity;

	char cDeltaHum = '|';
	if (humidity < aRecord->outsideHumidLo)
	{
		aRecord->outsideHumidLo = humidity;
		aRecord->outsideHumidLoTime= epoch;
		cStatus = 'R';
		cDeltaHum = '*';
	}

	if (humidity > aRecord->outsideHumidHi)
	{
		aRecord->outsideHumidHi = humidity;
		aRecord->outsideHumidHiTime= epoch;
		cStatus = 'R';
		cDeltaHum = '*';
	}


	char cLo = '|';
	if (temp < aRecord->outsideTempLo)
	{
		aRecord->outsideTempLo = temp;
		aRecord->outsideTempLoTime = epoch;
		cStatus = 'R';
		cLo = '*';
	}

	char cHi = '|';
	if (temp > aRecord->outsideTempHi)
	{
		aRecord->outsideTempHi = temp;
		aRecord->outsideTempHiTime = epoch;
		cStatus = 'R';
		cHi = '*';
	}

	if (aRecord->lastArrivalTime) aRecord->deltaArrivalTime = now - aRecord->lastArrivalTime;
	aRecord->lastArrivalTime = now;

	if (channel == 1) oprintf(1,"temp %+.1f %.1f %.1f", temp, aRecord->outsideTempHi, aRecord->outsideTempLo);

	struct tm	ts_hi;
	struct tm	ts_lo;
	struct tm	ts_now;

	char  lobuf[20];
	char  hibuf[20];
	char  nowbuf[20];

	ts_lo = *localtime(&aRecord->outsideTempLoTime);
	ts_hi = *localtime(&aRecord->outsideTempHiTime);

	ts_now = *localtime(&epoch);

	strftime(lobuf,  sizeof(lobuf), "%m-%d %H:%M", &ts_lo);
	strftime(hibuf,  sizeof(hibuf), "%m-%d %H:%M", &ts_hi);
	strftime(nowbuf, sizeof(nowbuf), "%m-%d %H:%M", &ts_now);

	char output[150];

	sprintf(output, "%c dev=0x%04X ch=%02d %lu  %s %+5.1f degC %c lo %s %+5.1f degC %c hi  %s %+5.1f degC %c hum=%02d%% | %lu mS \n",
		cStatus,
		device, channel,
	    epoch, nowbuf, temp,
		cLo, lobuf, aRecord->outsideTempLo,
		cHi, hibuf, aRecord->outsideTempHi,
		cDeltaHum, humidity,
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

