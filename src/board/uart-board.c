/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
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
 */
#include <stm32l0xx_hal.h>

#include "board.h"
#include "uart-board.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT 10

struct {
    Uart_t *obj;
    UART_HandleTypeDef handle;
    uint8_t rxData;
    uint8_t txData;
} McuUarts[2];

void UartMcuInit(Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx)
{
    obj->UartId = uartId;

    if (uartId == UART_1)
    {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_ENABLE();

        GpioInit(&obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF4_USART1);
        GpioInit(&obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF4_USART1);

        McuUarts[uartId].handle.Instance = USART1;
        McuUarts[uartId].obj = obj;
    }
    else if (uartId == UART_2)
    {
        __HAL_RCC_USART4_FORCE_RESET();
        __HAL_RCC_USART4_RELEASE_RESET();
        __HAL_RCC_USART4_CLK_ENABLE();

        GpioInit(&obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF6_USART4);
        GpioInit(&obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF6_USART4);

        McuUarts[uartId].handle.Instance = USART4;
        McuUarts[uartId].obj = obj;
    }
    else
    {
        assert_param(false);
    }
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    UART_HandleTypeDef *uartHandle;

    if (obj->UartId != UART_1 && obj->UartId != UART_2)
    {
        assert_param(false);
    }

    uartHandle = &McuUarts[obj->UartId].handle;
    uartHandle->Init.BaudRate = baudrate;

    if (mode == TX_ONLY)
    {
        if (obj->FifoTx.Data == NULL)
        {
            assert_param(false);
        }
        uartHandle->Init.Mode = UART_MODE_TX;
    }
    else if (mode == RX_ONLY)
    {
        if( obj->FifoRx.Data == NULL )
        {
            assert_param( false );
        }
        uartHandle->Init.Mode = UART_MODE_RX;
    }
    else if ( mode == RX_TX )
    {
        if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
        {
            assert_param( false );
        }
        uartHandle->Init.Mode = UART_MODE_TX_RX;
    }
    else
    {
        assert_param( false );
    }

    if( wordLength == UART_8_BIT )
    {
        uartHandle->Init.WordLength = UART_WORDLENGTH_8B;
    }
    else if( wordLength == UART_9_BIT )
    {
        uartHandle->Init.WordLength = UART_WORDLENGTH_9B;
    }

    switch( stopBits )
    {
        case UART_2_STOP_BIT:
            uartHandle->Init.StopBits = UART_STOPBITS_2;
            break;
        case UART_1_5_STOP_BIT:
            uartHandle->Init.StopBits = UART_STOPBITS_1_5;
            break;
        case UART_1_STOP_BIT:
        default:
            uartHandle->Init.StopBits = UART_STOPBITS_1;
            break;
    }

    if( parity == NO_PARITY )
    {
        uartHandle->Init.Parity = UART_PARITY_NONE;
    }
    else if( parity == EVEN_PARITY )
    {
        uartHandle->Init.Parity = UART_PARITY_EVEN;
    }
    else
    {
        uartHandle->Init.Parity = UART_PARITY_ODD;
    }

    if( flowCtrl == NO_FLOW_CTRL )
    {
        uartHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }
    else if( flowCtrl == RTS_FLOW_CTRL )
    {
        uartHandle->Init.HwFlowCtl = UART_HWCONTROL_RTS;
    }
    else if( flowCtrl == CTS_FLOW_CTRL )
    {
        uartHandle->Init.HwFlowCtl = UART_HWCONTROL_CTS;
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL )
    {
        uartHandle->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }

    uartHandle->Init.OverSampling = UART_OVERSAMPLING_16;

    if( HAL_UART_Init( uartHandle ) != HAL_OK )
    {
        assert_param(false);
    }

    if (obj->UartId == UART_1)
    {
        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        HAL_UART_Receive_IT(uartHandle, &McuUarts[obj->UartId].rxData, 1);
    }
    else if (obj->UartId == UART_2)
    {
        HAL_NVIC_SetPriority(USART4_5_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART4_5_IRQn);
        HAL_UART_Receive_IT(uartHandle, &McuUarts[obj->UartId].rxData, 1);
    }
}

void UartMcuDeInit( Uart_t *obj )
{
    UART_HandleTypeDef *uartHandle;

    if (obj->UartId != UART_1 && obj->UartId != UART_2)
    {
        assert_param(false);
    }

    uartHandle = &McuUarts[obj->UartId].handle;
    HAL_UART_DeInit(uartHandle);
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    UART_HandleTypeDef *uartHandle;
    uint8_t *txData;

    if (obj->UartId != UART_1 && obj->UartId != UART_2)
    {
        assert_param(false);
    }

    uartHandle = &McuUarts[obj->UartId].handle;
    txData = &McuUarts[obj->UartId].txData;

    CRITICAL_SECTION_BEGIN( );
    *txData = data;

    if( IsFifoFull( &obj->FifoTx ) == false )
    {
        FifoPush( &obj->FifoTx, *txData );

        // Trig UART Tx interrupt to start sending the FIFO contents.
        __HAL_UART_ENABLE_IT( uartHandle, UART_IT_TC );

        CRITICAL_SECTION_END( );
        return 0; // OK
    }

    CRITICAL_SECTION_END( );
    return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    if (obj->UartId != UART_1 && obj->UartId != UART_2)
    {
        assert_param(false);
    }

    CRITICAL_SECTION_BEGIN( );

    if( IsFifoEmpty( &obj->FifoRx ) == false )
    {
        *data = FifoPop( &obj->FifoRx );
        CRITICAL_SECTION_END( );
        return 0;
    }
    CRITICAL_SECTION_END( );
    return 1;
}

uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutBuffer( obj, buffer, size );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        uint8_t retryCount;
        uint16_t i;

        for( i = 0; i < size; i++ )
        {
            retryCount = 0;
            while( UartPutChar( obj, buffer[i] ) != 0 )
            {
                retryCount++;

                // Exit if something goes terribly wrong
                if( retryCount > TX_BUFFER_RETRY_COUNT )
                {
                    return 1; // Error
                }
            }
        }
        return 0; // OK
    }
}

uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
{
    uint16_t localSize = 0;

    while( localSize < size )
    {
        if( UartGetChar( obj, buffer + localSize ) == 0 )
        {
            localSize++;
        }
        else
        {
            break;
        }
    }

    *nbReadBytes = localSize;

    if( localSize == 0 )
    {
        return 1; // Empty
    }
    return 0; // OK
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *handle )
{
    Uart_t *obj;
    uint8_t *txData;

    if (&McuUarts[0].handle == handle)
    {
        obj = McuUarts[0].obj;
        txData = &McuUarts[0].txData;
    }
    else if (&McuUarts[1].handle == handle)
    {
        obj = McuUarts[1].obj;
        txData = &McuUarts[1].txData;
    }
    else
    {
        assert_param(false);
    }

    if( IsFifoEmpty( &obj->FifoTx ) == false )
    {
        *txData = FifoPop( &obj->FifoTx );
        //  Write one byte to the transmit data register
        HAL_UART_Transmit_IT( handle, txData, 1 );
    }

    if( obj->IrqNotify != NULL )
    {
        obj->IrqNotify( UART_NOTIFY_TX );
    }
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
{
    Uart_t *obj;
    uint8_t *rxData;

    if (&McuUarts[0].handle == handle)
    {
        obj = McuUarts[0].obj;
        rxData = &McuUarts[0].rxData;
    }
    else if (&McuUarts[1].handle == handle)
    {
        obj = McuUarts[1].obj;
        rxData = &McuUarts[1].rxData;
    }
    else
    {
        assert_param(false);
    }

    if( IsFifoFull( &obj->FifoRx ) == false )
    {
        // Read one byte from the receive data register
        FifoPush( &obj->FifoRx, *rxData );
    }

    if( obj->IrqNotify != NULL )
    {
        obj->IrqNotify( UART_NOTIFY_RX );
    }

    HAL_UART_Receive_IT( handle, rxData, 1 );
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle )
{
    uint8_t *rxData;

    if (&McuUarts[0].handle == handle)
    {
        rxData = &McuUarts[0].rxData;
    }
    else if (&McuUarts[1].handle == handle)
    {
        rxData = &McuUarts[1].rxData;
    }
    else
    {
        assert_param(false);
    }

    HAL_UART_Receive_IT( handle, rxData, 1 );
}

void USART1_IRQHandler( void )
{
    HAL_UART_IRQHandler( &McuUarts[0].handle );
}

void USART4_5_IRQHandler( void )
{
    HAL_UART_IRQHandler( &McuUarts[1].handle );
}
