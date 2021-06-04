/*!
 * \file      gps-board.c
 *
 * \brief     CXD5603GF GPS receiver driver implementation
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
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Alexey Ryabov
 */

#include <string.h>

#include "board.h"
#include "delay.h"
#include "gpio.h"
#include "gps.h"
#include "uart.h"
#include "gps-board.h"
#include "board-config.h"

#define GPS_RX_BUF_SIZE 128

static void GpsUartIrqNotify(UartNotifyId_t id);

static Gpio_t LvlShifter;

static Gpio_t GpsPowerEn;

static Gpio_t GpsReset;

static char GpsRxBuf[GPS_RX_BUF_SIZE];
static uint16_t GpsRxSize;
uint16_t GpsStatus; // for debugging purpose

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
static uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
static volatile uint8_t NmeaStringSize = 0;

static volatile bool IsGpsDataParsed;

static bool GpsSendCommand( char *cmd, uint32_t delay )
{
    char *ok;

    UartPutBuffer( &Uart2, ( uint8_t * )cmd, strlen( cmd ) );
    DelayMs(delay);
    UartGetBuffer( &Uart2, ( uint8_t * )GpsRxBuf, sizeof( GpsRxBuf ) - 1, &GpsRxSize );
    GpsRxBuf[GpsRxSize] = '\0';
    ok = strstr( GpsRxBuf, "Done" );

    return ok != NULL;
}

void GpsMcuOnPpsSignal( void *context )
{

}

void GpsMcuInvertPpsTrigger( void )
{

}

void GpsMcuInit( void )
{
    GpioInit( &LvlShifter, LVL_SHIFTER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &GpsPowerEn, GPS_POWER_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &GpsReset, GPS_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
}

void GpsMcuStart( void )
{
    const uint32_t delay = 400; // ms
    char *commands[] =
    {
        "@VER\r\n",
        "@WUP\r\n",
        "@BSSL 1\r\n",
        "@GSOP 1 1000 0\r\n",
        "@GNS 3\r\n",
        "@GSP\r\n",
    };
    
    UartInit( &Uart2, UART_2, UART2_TX, UART2_RX );
    UartConfig( &Uart2, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

    GpioWrite( &GpsPowerEn, 1 );
    DelayMs( 200 );

    GpioWrite( &GpsReset, 0 );
    DelayMs( 200 );
    GpioWrite( &GpsReset, 1 );
    DelayMs( 400 );

    GpsStatus = 0x0;

    for( uint32_t i = 0; i < sizeof( commands ) / sizeof( commands[0] ); i++ )
    {
        if( GpsSendCommand( commands[i], delay ) )
            GpsStatus |= 1 << i;
    }

    Uart2.IrqNotify = GpsUartIrqNotify;
}

void GpsMcuStop( void )
{
    uint32_t delay = 400; // ms
    Uart2.IrqNotify = NULL;

    if( GpsSendCommand( "@GSTP\r\n", delay ) )
        GpsStatus |= 0x4000;
    delay = 1000;
    if( GpsSendCommand( "@BUP\r\n", delay ) )
        GpsStatus |= 0x8000;
    GpioWrite( &GpsPowerEn, 0 );

    UartDeInit( &Uart2 );
}

void GpsMcuProcess( void )
{

}

void GpsUartIrqNotify( UartNotifyId_t id )
{
    uint8_t data;

    if( id == UART_NOTIFY_RX )
    {
        if( UartGetChar( &Uart2, &data ) == 0 )
        {
            if( ( data == '$' ) || ( NmeaStringSize >= 127 ) )
            {
                NmeaStringSize = 0;
            }

            NmeaString[NmeaStringSize++] = data;

            if( data == '\n' )
            {
                NmeaString[NmeaStringSize++] = '\0';
                if ( GpsParseGpsData( ( int8_t * )NmeaString, NmeaStringSize ) )
                    IsGpsDataParsed = 1;
            }
        }
    }
}

uint8_t GpsMcuIsDataParsed()
{
    uint8_t isDataParsed;

    CRITICAL_SECTION_BEGIN( );
    isDataParsed = IsGpsDataParsed;
    IsGpsDataParsed = 0;
    CRITICAL_SECTION_END( );

    return isDataParsed;    
}