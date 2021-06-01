/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "gpio.h"
#include "uart.h"

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 0

/*!
 * Board MCU pins definitions
 */
#define LED_RED PA_11
#define LED_GREEN PA_12
#define LED_BLUE PA_8
#define UART1_TX PA_9
#define UART1_RX PA_10
#define UART2_TX PC_10
#define UART2_RX PC_11
#define LVL_SHIFTER PC_6
#define GPS_RESET PB_2
#define GPS_POWER_EN PC_4
#define CHARGE_OUT PB_8
#define CHARGE_IN PB_1
#define ADC_VBAT PB_0

#define RADIO_RESET PB_10
#define RADIO_MOSI PB_15
#define RADIO_MISO PB_14
#define RADIO_SCLK PB_13
#define RADIO_NSS PB_12
#define RADIO_DIO_0 PB_11
#define RADIO_DIO_1 PC_13
#define RADIO_DIO_2 PB_9
#define RADIO_DIO_3 PB_4
#define RADIO_DIO_4 PB_3
#define RADIO_DIO_5 PA_15
#define RADIO_ANT_SWITCH PA_1
#define LORA_TCXO_OE PD_7
#define LORA_OSC_SEL PC_1

#define OSC_LSE_IN PC_14
#define OSC_LSE_OUT PC_15

#define OSC_HSE_IN PH_0
#define OSC_HSE_OUT PH_1

#define SWCLK PA_14
#define SWDAT PA_13

#define I2C_SCL PB_8
#define I2C_SDA PB_9

extern Gpio_t LedRed;
extern Gpio_t LedGreen;
extern Gpio_t LedBlue;

/*
 * MCU objects
 */
extern Uart_t Uart1;
extern Uart_t Uart2;

void BoardProcess( void );

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
