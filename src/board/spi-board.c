/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <stm32l0xx.h>
#include "board.h"
#include "spi-board.h"

/*!
 * \brief  Find First Set
 *         This function identifies the least significant index or position of the
 *         bits set to one in the word
 *
 * \param [in]  value  Value to find least significant index
 * \retval bitIndex    Index of least significat bit at one
 */
__STATIC_INLINE uint8_t __ffs( uint32_t value )
{
    return( uint32_t )( 32 - __CLZ( value & ( -value ) ) );
}

SPI_HandleTypeDef Spi1Handle;
SPI_HandleTypeDef Spi2Handle;

static SPI_HandleTypeDef *SpiHandleById(SpiId_t spiId)
{
    if (spiId == SPI_1)
        return &Spi1Handle;

    if (spiId == SPI_2)
        return &Spi2Handle;

    assert_param(0);
    return (SPI_HandleTypeDef *)0;
}

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    uint32_t af = 0;

    obj->SpiId = spiId;

    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);

    if (obj->SpiId == SPI_1)
    {
        spiHandle->Instance = SPI1;
    }
    else if (obj->SpiId == SPI_2)
    {
        __HAL_RCC_SPI2_FORCE_RESET( );
        __HAL_RCC_SPI2_RELEASE_RESET( );
        __HAL_RCC_SPI2_CLK_ENABLE( );

        spiHandle->Instance = SPI2;

        // Note SPI2 can only use AF0
        af = GPIO_AF0_SPI2;
    }

    GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, af );
    GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, af );
    GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, af );

    if( nss != NC )
    {
        GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, af );
    }
    else
    {
        spiHandle->Init.NSS = SPI_NSS_SOFT;
    }

    if( nss == NC )
    {
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 0 );
    }
    else
    {
        SpiFormat( obj, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 1 );
    }

    SpiFrequency( obj, 10000000 );

    HAL_SPI_Init(spiHandle);

    //AcSiP(+), for SPI speeds up
    __HAL_SPI_ENABLE(spiHandle);
}

void SpiDeInit( Spi_t *obj )
{
    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);

    HAL_SPI_DeInit( spiHandle );

    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);

    spiHandle->Init.Direction = SPI_DIRECTION_2LINES;
    if( bits == SPI_DATASIZE_8BIT )
    {
        spiHandle->Init.DataSize = SPI_DATASIZE_8BIT;
    }
    else
    {
        spiHandle->Init.DataSize = SPI_DATASIZE_16BIT;
    }
    spiHandle->Init.CLKPolarity = cpol;
    spiHandle->Init.CLKPhase = cpha;
    spiHandle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiHandle->Init.TIMode = SPI_TIMODE_DISABLE;
    spiHandle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHandle->Init.CRCPolynomial = 7;

    if( slave == 0 )
    {
        spiHandle->Init.Mode = SPI_MODE_MASTER;
    }
    else
    {
        spiHandle->Init.Mode = SPI_MODE_SLAVE;
    }
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    uint32_t divisor;
    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);

    divisor = SystemCoreClock / hz;

    // Find the nearest power-of-2
    divisor = divisor > 0 ? divisor-1 : 0;
    divisor |= divisor >> 1;
    divisor |= divisor >> 2;
    divisor |= divisor >> 4;
    divisor |= divisor >> 8;
    divisor |= divisor >> 16;
    divisor++;

    divisor = __ffs( divisor ) - 1;

    divisor = ( divisor > 0x07 ) ? 0x07 : divisor;

    spiHandle->Init.BaudRatePrescaler = divisor << 3;

    //AcSiP(+), for SPI speeds up, 32MHz / 4 = 8MHz
    spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
}

FlagStatus SpiGetFlag( Spi_t *obj, uint16_t flag )
{
    FlagStatus bitstatus = RESET;
    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);

    // Check the status of the specified SPI flag
    if( ( spiHandle->Instance->SR & flag ) != ( uint16_t )RESET )
    {
        // SPI_I2S_FLAG is set
        bitstatus = SET;
    }
    else
    {
        // SPI_I2S_FLAG is reset
        bitstatus = RESET;
    }
    // Return the SPI_I2S_FLAG status
    return  bitstatus;
}

void SPI_SendData8( SPI_TypeDef *SPIx, uint8_t Data )
{
    uint32_t spixbase = 0x00;

    spixbase = (uint32_t)SPIx;
    spixbase += 0x0C;

    *(__IO uint8_t *) spixbase = Data;
}


uint8_t SPI_ReceiveData8 ( SPI_TypeDef *SPIx )
{
    uint32_t spixbase = 0x00;

    spixbase = (uint32_t)SPIx;
    spixbase += 0x0C;

    return *(__IO uint8_t *) spixbase;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    SPI_HandleTypeDef *spiHandle = SpiHandleById(obj->SpiId);
    uint8_t rxData = 0;

    //AcSiP(-) for SPI speeds up
    //if( ( obj == NULL ) || ( obj->Spi.Instance ) == NULL )
    //{
    //    assert_param( FAIL );
    //}

    //AcSiP(-), for SPI speeds up
    //__HAL_SPI_ENABLE( &obj->Spi );

    while( SpiGetFlag( obj, SPI_FLAG_TXE ) == RESET );
    //obj->Spi.Instance->DR = ( uint16_t ) ( outData & 0xFF );
    SPI_SendData8(spiHandle->Instance, (uint8_t)(outData & 0xFF));

    while( SpiGetFlag( obj, SPI_FLAG_RXNE ) == RESET );
    //rxData = ( uint16_t ) obj->Spi.Instance->DR;
    rxData = SPI_ReceiveData8(spiHandle->Instance);

    return( rxData );
}
