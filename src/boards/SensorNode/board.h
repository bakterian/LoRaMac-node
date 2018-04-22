/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "mag3110.h"
#include "mma8451.h"
#include "mpl3115.h"
#include "sx9500.h"
#include "gps.h"
#include "gps-board.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "uart-board.h"
#include "dwt_stm32_delay.h"
#include "dht22.h"

#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

/*!
 * Define indicating if an external IO expander is to be used
 */ // A external IO expander is not connected
//#define BOARD_IOE_EXT

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif


/*!
 * Board MCU pins definitions
 * ALWAYS VERIFY IF PIN IS NOT ALREADY USED ON NUCLEO BOARD
 */

#define RADIO_RESET                                 PB_6

#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13
#define RADIO_NSS                                   PA_4

#define RADIO_DIO_0                                 PB_5
#define RADIO_DIO_1                                 PC_1
#define RADIO_DIO_2                                 PC_0
#define RADIO_DIO_3                                 PC_2
#define RADIO_DIO_4                                 PC_3
#define RADIO_DIO_5                                 PB_12

#define DHT22_IO_IN									PB_7
#define LED_1                                       PA_5
#define DHT22_IO_OUT								PA_1


/*!
 * Board MCU pins which are unavailable (at least without nucleo reworks)
 */

#define NUCLEO_USER_BUTTON							PC_13

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PD_0
#define OSC_HSE_OUT                                 PD_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9

#define UART_TX                                     PA_2
#define UART_RX                                     PA_3

#define UART3_TX                                    PC_10
#define UART3_RX                                    PC_11

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define JNTRST										PB_4
#define JTDO										PB_3
#define JTDI										PA_15


/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;
extern Gpio_t Led3;
extern Gpio_t Led4;

/*!
 * MCU objects
 */
extern I2c_t I2c;
extern Uart_t Uart2;
#if defined( USE_USB_CDC )
extern Uart_t UartUsb;
#endif

/*!
 * DHT22 GPIO pins objects
 */
extern Gpio_t InDht22GPIO;
extern Gpio_t OutDht22GPIO;

/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

#endif // __BOARD_H__
