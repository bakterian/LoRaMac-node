/**
  ******************************************************************************
  * @file    dht22.c
  * @author  Marcin K.
  * @version V1.0
  * @date    16.03.2018
  * @brief   
  ******************************************************************************
  */

#include "board.h"
#include "dht22.h"

static uint8_t 				m_au8DataBuffer[DATA_BUFFER_SIZE];
static uint8_t	 			m_au8PulseContainer[VOLTAGE_PULSE_COUNT];

void InitDht22(Gpio_t* dht22GPIO, PinNames dht22PinName)
{
	memset(&m_au8PulseContainer,0x00U,VOLTAGE_PULSE_COUNT);
	memset(&m_au8DataBuffer,0x00U,DATA_BUFFER_SIZE);

	// setting talker pin as input with the the internal pull-up
    GpioInit(dht22GPIO, dht22PinName, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
}

uint8_t GetDht22Measurements(Gpio_t* dht22GPIO, uint8_t* pResults)
{
	ERRORTYPE eRet = ET_OK;

	//used to wait in the pre-init stage for 2000 ms

	eRet = ReceiveData(dht22GPIO,dht22GPIO->pin);

	if(eRet == ET_OK)
	{
		eRet = EvaluateData();
	}

	for (int var = 0; var < MEAS_DATA_COUNT; ++var)
	{
		pResults[var] = m_au8DataBuffer[var];
	}


	return (uint8_t)eRet;
}

ERRORTYPE ReceiveData(Gpio_t* dht22GPIO, PinNames dht22PinName)
{
	ERRORTYPE eRet = ET_OK;

	// setting talker pin as output, talker output pin should stay low
    GpioInit( dht22GPIO, dht22PinName, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioWrite( dht22GPIO,0);

    DelayMs(20U); //min is 1 ms, max is 20 ms

    // set pin to high in order to finish the initial handshake
    GpioWrite( dht22GPIO,1);

    //delay task for 40us, accepted from 30 to 200 us
    DWT_Delay_us(TIMEOUT_AFTER_HANDSHAKE_US);

    GpioInit( dht22GPIO, dht22PinName, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    DWT_Delay_us(TIMEOUT_BEFORE_READ_US);	// delay a bit to let the sensor pull the data line low.

	uint32_t pulseCount = getPulseDuration(dht22GPIO,eLow);
	if (PULSE_COUNT_TIMEDOUT == pulseCount) // expecting a 80us low pulse from DHT
	{
		eRet = ET_NOK;
	}

	uint32_t pulseCount2 = getPulseDuration(dht22GPIO,eHigh);
	if((eRet == ET_OK) && 								// expecting a 80us high pulse from DHT
	   (PULSE_COUNT_TIMEDOUT == pulseCount2))
	{
		eRet = ET_NOK;
	}

	if(eRet == ET_OK)
	{
		// Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
		// microsecond low pulse followed by a variable length high pulse.  If the
		// high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
		// then it's a 1.

		for (uint8_t u8Loop = 0 ; u8Loop < VOLTAGE_PULSE_COUNT ; u8Loop+=2)
		{
			m_au8PulseContainer[u8Loop]    = getPulseDuration(dht22GPIO,eLow);
			m_au8PulseContainer[u8Loop+1U] = getPulseDuration(dht22GPIO,eHigh);
		}
	}

	return (eRet);
}

ERRORTYPE EvaluateData()
{
	ERRORTYPE eRet = ET_OK;

	// Inspect pulses and determine which ones are 0 (high state cycle count < low
	// state cycle count), or 1 (high state cycle count > low state cycle count).
	for (uint8_t u8Loop = 0 ; u8Loop < (VOLTAGE_PULSE_COUNT/2) ; ++u8Loop)
	{
		uint8_t u8lowCycles  = m_au8PulseContainer[2*u8Loop];
		uint8_t u8HighCycles = m_au8PulseContainer[(2*u8Loop)+1];

		if ((u8lowCycles == PULSE_COUNT_TIMEDOUT) || (u8HighCycles == PULSE_COUNT_TIMEDOUT))
		{
			printf("DHT Talker| Pulse count mismatch lowCycles: %u highCycles,\r\n", u8lowCycles,u8HighCycles );
			eRet = ET_NOK;
			break;
		}

		m_au8DataBuffer[u8Loop/8] <<= 1; //move previous result

		if (u8HighCycles > u8lowCycles) //comparing the low and high cycle times to see if the bit is a 0 or 1.
		{ // High cycles are greater than 50us low cycle count, must be a 1.
			m_au8DataBuffer[u8Loop/8] |= 1;
		}
		// Else high cycles are less than (or equal to, a weird case) the 50us low
		// cycle count so this must be a zero.
	}

	// Check we read 40 bits and that the checksum matches.
	if ((eRet == ET_OK) &&
	  (m_au8DataBuffer[4] != ((m_au8DataBuffer[0] + m_au8DataBuffer[1] +
							  m_au8DataBuffer[2] + m_au8DataBuffer[3]) & 0xFF)))
	{
		eRet = ET_CRC_ERROR;
		printf("DHT Talker| Checksum mismatch\r\n");
	}

	return (eRet);
}


uint32_t getPulseDuration(Gpio_t* dht22GPIO, VoltagePulseType ePulseType)
{
	uint32_t u32count = 0;

	while (GpioRead(dht22GPIO) == ePulseType)
	{
	  ++u32count;

	  if (u32count >= MAX_PULSE_COUNT )
	  {
		 u32count =  0; // Exceeded timeout.
		 break;
	  }
	}

	return u32count;
}
