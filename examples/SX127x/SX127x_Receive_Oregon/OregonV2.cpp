#include "OregonV2.h"

bool bPrintIfChangeFound = true;  // only if a change is detected then console is  printed.

//---------------------------------------
guinessRecord * locateDeviceStats(uint16_t device, uint8_t channel)
{
   static guinessRecord *first = NULL;
   guinessRecord *ret = first;
   guinessRecord *lastRet = NULL;

	uint32_t devChannel = device;
	devChannel = (devChannel << 4) + channel;

    if (!first)
        {
           //Serial.printf("NEW = 0x%X\n", devChannel);
           ret = (guinessRecord *) malloc (sizeof (guinessRecord));
           first = ret;
           memset(ret, 0, sizeof(guinessRecord));
init:
          time_t tick;
          time(&tick);

           ret->devChannel = devChannel;
           ret->outsideTempHi = -1000.;
           ret->outsideTempLo= +1000.;
           ret->outsideRainRate = -1.;
           ret->outsideTempHiTime = tick;
           ret->outsideTempLoTime = tick;
           ret->outsideWindTime = tick;
		   ret->outsideWindAvgTime = tick;
		   ret->outsideWindPeakTime = tick;
		   ret->outsideRainRateTime = tick;
		   ret->outsideRainTotalTime = tick;
           return ret;
        }
    else
        {
			//Serial.printf("found = 0x%X\n", devChannel);
            lastRet = ret;
            while( ret && ret->devChannel != devChannel)
            {
              lastRet = ret;
              ret = ret->next;
            }

            if (!ret)
            {
                ret = (guinessRecord *) malloc (sizeof (guinessRecord));
                memset(ret, 0, sizeof(guinessRecord));
                lastRet->next = ret;
                goto init;
            }
        }
    return ret;
}

//---------------------------------------
float windchill (float speed, float temperature)
{
    //Serial.printf("%s : speed = %f temperature = %f\n", __FUNCTION__, speed, temperature);
    float chill = 13.12 + 0.6215 * temperature -  11.37 * pow(speed,0.16) + 0.3965 * temperature * pow(speed,0.16);
    return chill;
}

//---------------------------------------------------------------------------------------------------
//Procedure for calculating CRC8 and checksum for Oregon sensors
//oregon_data - a pointer to a code message
//CCIT_POLY - CRC generating polynomial
//CCIT_START - initial CRC value
//p_length - packet length
//v3 - set to 1 if the third version of the protocol
//---------------------------------------------------------------------------------------------------

static inline uint8_t getNibble( OREGON_COMMON* oregon_data, uint8_t x)
{
	uint8_t result;
	result = oregon_data->uchar8x16.array[x/2];
	result = (x & 1) ? result >> 4 : result & 0xF;
	//Serial.printf("%s [%d] = 0x%02x\n", __FUNCTION__, x, result);

	return result;
}

uint32_t numPacketsTested = 0;
uint32_t numPacketsFailed = 0;
uint32_t lastFailedCRCdeviceChan = 0;

//---------------------------------------

void ShiftLeft128(uint64_t dest[])
{
    dest[0] <<= 1;
    uint64_t hold;
    hold = dest[1] & HI_BIT64;
    hold = !!hold;
    dest[0] |= hold;
    dest[1] <<= 1;
}
//---------------------------------------

// keep shifting until MSb is a one.
void leftJustify128TillOne(uint64_t dest[])
{
    if (dest[0] == 0 && dest[1] == 0) return;  // stop infinity

    while( (dest[0] & HI_BIT64) == 0)
    {
        ShiftLeft128(dest);
    }
}
//---------------------------------------

void uint128DumpInBinary(uint64_t dest[], char *msg)
{
#if FULL_DEBUG

    //Serial.printf("%8s : ");
    Serial.print(msg);
    Serial.print(" ");

    Serial.print(dest[0], HEX);
    Serial.print("^");

    Serial.print(dest[1], HEX);
    Serial.print(" :  ");

    print64(dest[0]);
    Serial.print(" | ");
    print64ln(dest[1]);
#endif
}

//---------------------------------------
void uint128DumpInHex(uint64_t dest[], char *msg)
{
Serial.print(dest[0],HEX);Serial.print('.');Serial.println(dest[1],HEX);

}
//------------------------------------------------------------
static const uint8_t reverseNibble []=
{ 0, 8, 4, 12, 2, 5, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15};

void revStringNibbles(uint8_t *what, unsigned int numBits)
{
    for (unsigned int i = 0; i < (numBits/8 +1); i++)
    {
        uint8_t temp = 0;
        temp  = reverseNibble[what[i] >> 4] << 4;
        temp |= reverseNibble[what[i] & 0xF];
        what[i] = temp;
    }
}

//------------------------------------------------------------
uint64_t revEndian64ByBits (uint64_t *what)
{
    uint64_t output = 0;
    uint64_t input = *what;
    uint64_t temp;

    for (unsigned int i = 0; i < 64; i++)  // 16 nibbles = 64 bits.
    {
        output <<= 1;
        temp  = input & 0x1;
        output |= temp;
        input  >>= 1;
    }
    *what = output;
    return output;
}
//------------------------------------------------------------




bool check_oregon_crcsum(OREGON_COMMON* od, byte CCIT_POLY, byte CCIT_START, byte p_length, bool v3)
{

  numPacketsTested++;

  //byte* pp = oregon_data;
  byte calcSUM = 0, calcCRC = CCIT_START,  given_sum, given_crc;
  int x;

  for(x=0; x < p_length - 4; x++)
  {
    uint8_t hold;
    hold = getNibble(od, x);
    calcSUM += hold;  //*pp;
    if ( v3 || (x != 5 && x != 6))
    {
      calcCRC ^= hold;  //*pp;
      for(byte i = 0; i < 4; i++)
        if(calcCRC & 0x80) calcCRC = (calcCRC << 1) ^ CCIT_POLY;
        else calcCRC <<= 1;
    }
    //pp++;
  }

  for(byte i = 0; i < 4; i++)
    if(calcCRC & 0x80) calcCRC = (calcCRC << 1) ^ CCIT_POLY;
    else calcCRC <<= 1;

  given_sum = getNibble(od, x+0) + getNibble(od, x+1)*0x10;
  given_crc   = getNibble(od, x+2) + getNibble(od, x+3)*0x10;

  Serial.printf("SUM %s given = 0x%X vs calc = 0x%X\n", given_sum == calcSUM ? "PASS": "FAIL", given_sum, calcSUM);
  Serial.printf("CRC %s given = 0x%X vs calc = 0x%X\n", given_crc == calcCRC ? "PASS": "FAIL", given_crc, calcCRC);

  yield();
  bool pass = (given_crc == calcCRC && given_sum == calcSUM)? true : false;

  if (!pass)
  {
  	numPacketsFailed++;

	uint32_t device = od->sensid3 * 0x1000 + od->sensid2 * 0x100 + od->sensid1 * 0x10 + od->sensid0;

  	lastFailedCRCdeviceChan = device << 4 + od->channel;
  }

  return pass;
}

//---------------------------------------------------------------------------------------------------
//Procedure for calculating CRC8 and checksum for Oregon sensors
//oregon_data - a pointer to a code message
//CCIT_POLY - CRC generating polynomial
//CCIT_START - initial CRC value
//p_length - packet length
//---------------------------------------------------------------------------------------------------
bool check_own_crcsum(OREGON_COMMON* od, byte p_length)

{
  //byte* pp = oregon_data;
  byte cksum = 0, crc = 0,  recived_cksum, recived_crc;
  int x;

  for(x=0; x < p_length - 4; x++)
  {
  	uint8_t hold;
  	hold = getNibble(od, x);
    cksum += hold; //*pp;
    crc ^= hold;   //*pp;
    for(byte i = 0; i < 4; i++)
      if(crc & 0x80) crc = (crc << 1) ^ 7;
      else crc <<= 1;
    //pp++;
  }

  for(byte i = 0; i < 4; i++)
    if(crc & 0x80) crc = (crc << 1) ^ 7;
    else crc <<= 1;

  recived_cksum = getNibble(od, x+0) + getNibble(od, x+1)*0x10;
  recived_crc =   getNibble(od, x+2) + getNibble(od, x+3)*0x10;
  return (recived_crc == crc && recived_cksum == cksum)? 1 : 0;
}




