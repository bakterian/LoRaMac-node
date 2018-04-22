/**
  ******************************************************************************
  * @file    dht22.h
  * @author  Marcin K.
  * @version V1.0
  * @date    16.03.2018
  * @brief   
  ******************************************************************************
  */
#ifndef PERIPHERALS_DHT22_H_
#define PERIPHERALS_DHT22_H_

#ifdef __cplusplus
 extern "C" 
	{
#endif

#define DATA_BUFFER_SIZE 				5U
#define VOLTAGE_PULSE_COUNT 			80
#define MAX_PULSE_COUNT					0xFFFFFF00U
#define	PULSE_COUNT_TIMEDOUT			0U
#define	MEAS_DATA_COUNT					4U

#define TALKER_BIT						PB1
#define TALKER_PIN						PINB
#define TALKER_PORT						PORTB
#define TALKER_DDR						DDRB

#define TIMEOUT_AFTER_HANDSHAKE_US		40U
#define TIMEOUT_BEFORE_READ_US			10U
#define TIMEOUT_READ_START_US			80
#define	TICK_TIMEOUT_LAST_MEAS			2000
#define TICK_TIMEOUT_INIT_LOW_STATE		20

#define STATE_IDLE						0U
#define STATE_PRE_INIT					1U
#define STATE_RECEIVING_DATA			2U
#define STATE_EVALUATING_DATA			3U
#define STATE_ERRORS					4U

typedef enum VoltagePulseType
{
	eLow  = 0,
	eHigh = 1
}VoltagePulseType;

/*!
 * DHT22 Sensor Status return values
 */
typedef enum ERRORTYPE
{
	ET_OK 				= 0,
	ET_NOK 				= 1,
	ET_PENDING  		= 2,
	ET_ERROR			= 3,
	ET_ARG_TOO_BIG 		= 4,
	ET_I2C_BUSY 		= 5,
	ET_NULL_ARG			= 6,
	ET_NOT_INITIALIZED  = 7,
	ET_CRC_ERROR 		= 8
}ERRORTYPE;

/**
 * brief Delivers the measurement results
 * \return the measured temperature and humidity is returned.
 * \return non zero values in case of errors
 */
uint8_t GetDht22Measurements(Gpio_t* dht22GPIO, uint8_t* pResults);

/**
 * \return ET_OK - no problems were encountered
 * brief Initialize the DHT22 talker
 */
void InitDht22(Gpio_t* dht22GPIO, PinNames dht22PinName);

/**
 * brief Receives data from DHT22 - this is a critical section and task can not be interrupted
 * \return ET_OK - no problems were encountered
 * \return ET_OK_NOT - the were errors
 */
ERRORTYPE ReceiveData(Gpio_t* dht22GPIO, PinNames dht22PinName);

/**
 * brief Evaluating the received data - determines single bit values
 * \return ET_OK - no problems were encountered
 * \return ET_OK_NOT - the were errors
 */
ERRORTYPE EvaluateData();

/**
 * brief Provides the pulse duration
 * \return voltage pulse time duration in us
 * \return 0 in case of an timeout
 */
uint32_t getPulseDuration(Gpio_t* dht22GPIO, VoltagePulseType ePulseType);

#ifdef __cplusplus
	}
#endif

#endif /* PERIPHERALS_DHT22_H_ */
