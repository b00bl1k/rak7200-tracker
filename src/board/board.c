/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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

#include <stm32l0xx.h>
#include "adc.h"
#include "board.h"
#include "board-config.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "spi.h"
#include "sx1276-board.h"
#include "timer.h"
#include "uart.h"
#include "utilities.h"

/*!
 * Unique Devices IDs register set ( STM32L0xxx )
 */
#define ID1 ( 0x1FF80050 )
#define ID2 ( 0x1FF80054 )
#define ID3 ( 0x1FF80064 )

/*!
 * UART1 FIFO buffers size
 */
#define UART1_FIFO_TX_SIZE 128
#define UART1_FIFO_RX_SIZE 128

/*!
 * UART2 FIFO buffers size
 */
#define UART2_FIFO_TX_SIZE 1024
#define UART2_FIFO_RX_SIZE 1024

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL       4200  // mV
#define BATTERY_MIN_LEVEL       2400  // mV
#define BATTERY_SHUTDOWN_LEVEL  2300  // mV

#define BATTERY_LORAWAN_UNKNOWN_LEVEL   255
#define BATTERY_LORAWAN_MAX_LEVEL       254
#define BATTERY_LORAWAN_MIN_LEVEL       1
#define BATTERY_LORAWAN_EXT_PWR         0

/*!
 * Factory power supply
 */
#define VDDA_VREFINT_CAL ( ( uint32_t ) 3000 )  // mV

/*!
 * VREF calibration value
 */
#define VREFINT_CAL ( *( uint16_t* ) ( ( uint32_t ) 0x1FF80078 ) )

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                   4095

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

static void BoardDeInitPeriph( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag used to indicate if board is powered from the USB
 */
static bool UsbIsConnected = false;

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

static volatile uint8_t IsChargeStatusChanged = 1;

uint8_t Uart1TxBuffer[UART1_FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[UART1_FIFO_RX_SIZE];

uint8_t Uart2TxBuffer[UART2_FIFO_TX_SIZE];
uint8_t Uart2RxBuffer[UART2_FIFO_RX_SIZE];

/*!
 * LED GPIO pins objects
 */
Gpio_t LedRed;
Gpio_t LedGreen;
Gpio_t LedBlue;

/*
 * MCU objects
 */
Uart_t Uart1;
Uart_t Uart2;
Adc_t Adc;

/*
 * Stuff GPIO pin objects
 */
Gpio_t ChargeOut;
Gpio_t ChargeIn;

static void OnChargeStatusChanged( void *context )
{
    IsChargeStatusChanged = 1;
}

static void CheckChargeStatus( void )
{
    uint16_t status1, status2;
    bool usbStatus;

    GpioInit( &ChargeIn, CHARGE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ChargeOut, CHARGE_OUT, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    DelayMs(10);
    status1 = AdcReadChannel( &Adc, ADC_CHANNEL_9 );

    GpioInit( &ChargeOut, CHARGE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    DelayMs(10);
    status2 = AdcReadChannel( &Adc, ADC_CHANNEL_9 );

    // Revert pin configuration for external interrupts
    GpioInit( &ChargeIn, CHARGE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    if( status1 < 500 )
    {
        // Strong pull down - charge in progress
        GpioWrite( &LedRed, 0 );
        GpioWrite( &LedGreen, 1 );
        usbStatus = true;
    }
    else
    {
        if( status2 < 500 )
        {
            // Weak pull-down - charge is complete
            GpioWrite( &LedRed, 1 );
            GpioWrite( &LedGreen, 0 );
            usbStatus = true;
        }
        else
        {
            // UVLO state
            GpioWrite( &LedRed, 1 );
            GpioWrite( &LedGreen, 1 );
            usbStatus = false;
        }
    }

    if( UsbIsConnected != usbStatus )
    {
        UsbIsConnected = usbStatus;
        if( UsbIsConnected )
        {
            BoardInitPeriph( );
            LpmSetStopMode( LPM_APPLI_ID, LPM_DISABLE );
        }
        else
        {
            BoardDeInitPeriph( );
            LpmSetStopMode( LPM_APPLI_ID, LPM_ENABLE );
        }
    }
}

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{
    UartInit( &Uart1, UART_1, UART1_TX, UART1_RX );
    UartConfig( &Uart1, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
}

static void BoardDeInitPeriph( void )
{
    UartDeInit( &Uart1 );
}

void BoardInitMcu( void )
{
    Gpio_t ioPin;

    if( McuInitialized == false )
    {
        HAL_Init( );

        // LEDs
        GpioInit( &LedRed, LED_RED, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &LedGreen, LED_GREEN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &LedBlue, LED_BLUE, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

        GpioInit( &ChargeOut, CHARGE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ChargeIn, CHARGE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioSetInterrupt( &ChargeIn, IRQ_RISING_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, OnChargeStatusChanged );

        GpioInit( &ioPin, ADC_VBAT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

        SystemClockConfig( );

        RtcInit( );
        AdcInit(&Adc, NC);
        
        FifoInit( &Uart1.FifoTx, Uart1TxBuffer, sizeof( Uart1TxBuffer ) );
        FifoInit( &Uart1.FifoRx, Uart1RxBuffer, sizeof( Uart1RxBuffer ) );
        
        FifoInit( &Uart2.FifoTx, Uart2TxBuffer, sizeof( Uart2TxBuffer ) );
        FifoInit( &Uart2.FifoRx, Uart2RxBuffer, sizeof( Uart2RxBuffer ) );

        BoardUnusedIoInit( );

        // Disables OFF mode - Enables lowest power mode (STOP)
        LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
    }
    else
    {
        SystemClockReConfig( );
    }

    SpiInit( &SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );

    if( McuInitialized == false )
    {
        McuInitialized = true;

        SX1276IoDbgInit( );
        SX1276IoTcxoInit( );
    }
}

void BoardProcess( void )
{
    uint8_t isNeedCheckChargeStatus;
    CRITICAL_SECTION_BEGIN( );
    isNeedCheckChargeStatus = IsChargeStatusChanged;
    IsChargeStatusChanged = 0;
    CRITICAL_SECTION_END( );

    if( isNeedCheckChargeStatus == 1 )
    {
        CheckChargeStatus( );
    }
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint16_t BoardBatteryMeasureVoltage( void )
{
    uint16_t vref;
    uint16_t vdda;
    uint16_t vdiv;
    float batteryVoltage;

    vdiv = AdcReadChannel( &Adc, ADC_CHANNEL_8 );
    vref = AdcReadChannel( &Adc, ADC_CHANNEL_VREFINT );
    vdda = ( float )VDDA_VREFINT_CAL * ( float )VREFINT_CAL / ( float )vref;

    batteryVoltage = (float ) vdda * ( ( float )vdiv / ( float )ADC_MAX_VALUE );
    batteryVoltage = (batteryVoltage * 1.6) / 0.6;
    
    return ( uint16_t )batteryVoltage;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    if( GetBoardPowerSource( ) == USB_POWER )
    {
        batteryLevel = BATTERY_LORAWAN_EXT_PWR;
    }
    else
    {
        if( BatteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = BATTERY_LORAWAN_MAX_LEVEL;
        }
        else if( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel =
                ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
        {
            batteryLevel = 1;
        }
        else  // if( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = BATTERY_LORAWAN_UNKNOWN_LEVEL;
        }
    }
    return batteryLevel;
}

static void BoardUnusedIoInit( void )
{
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
    RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // Enable HSI
    __HAL_RCC_HSI_CONFIG( RCC_HSI_ON );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    // Enable PLL
    __HAL_RCC_PLL_ENABLE( );

    // Wait till PLL is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    // Select PLL as system clock source
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    // Wait till PLL is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

uint8_t GetBoardPowerSource( void )
{
    if( UsbIsConnected == false )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode( void)
{
    CRITICAL_SECTION_BEGIN( );

    BoardDeInitMcu( );

    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    // Enable Ultra low power mode
    HAL_PWREx_EnableUltraLowPower( );

    // Enable the fast wake up from Ultra low power mode
    HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    // Enter Stop Mode
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initilizes the peripherals
    BoardInitMcu( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
}

#if !defined ( __CC_ARM )

/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
    if (Uart1.IsInitialized)
        UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )count );
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
    size_t bytesRead = 0;
    while( UartGetBuffer( &Uart1, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
    // Echo back the character
    while( UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
    return bytesRead;
}

#else

#include <stdio.h>

// Keil compiler
int fputc( int c, FILE *stream )
{
    while( UartPutChar( &Uart1, ( uint8_t )c ) != 0 );
    return c;
}

int fgetc( FILE *stream )
{
    uint8_t c = 0;
    while( UartGetChar( &Uart1, &c ) != 0 );
    // Echo back the character
    while( UartPutChar( &Uart1, c ) != 0 );
    return ( int )c;
}

#endif

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
