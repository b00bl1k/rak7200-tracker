/*!
 * \file      main.c
 *
 * \brief     Performs a periodic uplink
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
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Alexey Ryabov
 */

#include <stdio.h>
#include "utilities.h"
#include "board.h"
#include "board-config.h"
#include "lpm-board.h"
#include "gps.h"
#include "systime.h"
#include "timer.h"

#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "CayenneLpp.h"

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

#define FIRMWARE_VERSION                            0x01000000 // 1.0.0.0

/*!
 * LoRaWAN default end-device class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            (5 * 60 * 1000)

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            2

#define GPS_TIMER_TIMEOUT                           (3 * 30 * 1000)

/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

static void OnMacProcessNotify( void );
static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size );
static void OnNetworkParametersChange( CommissioningParams_t* params );
static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn );
static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn );
static void OnJoinRequest( LmHandlerJoinParams_t* params );
static void OnTxData( LmHandlerTxParams_t* params );
static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params );
static void OnClassChange( DeviceClass_t deviceClass );
static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params );
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection );
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame( void );
static void StartTxProcess( LmHandlerTxEvents_t txEvent );
static void UplinkProcess( void );

static void OnTxPeriodicityChanged( uint32_t periodicity );
static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed );
static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity );

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context );

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
{
    .Buffer = AppDataBuffer,
    .BufferSize = 0,
    .Port = 0
};

static float AppGpsTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = NULL,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

static volatile uint8_t IsGpsTimeout = 0;

static volatile uint8_t IsJoinComplete = 0;

static TimerEvent_t GpsTimer;

static void OnGpsTimerTimeoutEvent( void *context )
{
    IsGpsTimeout = 1;
}

/*!
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    GpsInit( );

    TimerInit( &GpsTimer, OnGpsTimerTimeoutEvent );
    TimerSetValue( &GpsTimer, GPS_TIMER_TIMEOUT );

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        // Fatal error, endless loop.
        while ( 1 )
        {
        }
    }

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError( 20 );

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

    StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );

    while( 1 )
    {
        // Processes the LoRaMac events
        LmHandlerProcess( );

        // Process application uplinks management
        UplinkProcess( );

        BoardProcess( );

        CRITICAL_SECTION_BEGIN( );
        if( IsMacProcessPending == 1 )
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            // The MCU wakes up through events
            BoardLowPowerHandler( );
        }
        CRITICAL_SECTION_END( );
    }
}

static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    if( params->Status == LORAMAC_HANDLER_SUCCESS )
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
    IsJoinComplete = 1;
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    GpioWrite( &LedBlue, 1 );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    switch( appData->Port )
    {
        default:
            break;
    }
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        default:
            break;
    }
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset( );
    CayenneLppAddDigitalInput( channel++, 0 );

    if( GpsHasFix( ) == true )
    {
        double latitude = 0, longitude = 0;
        uint16_t altitudeGps = 0xFFFF;

        GpsGetLatestGpsPositionDouble( &latitude, &longitude );
        altitudeGps = GpsGetLatestGpsAltitude( ); // in m

        CayenneLppAddGps( channel++, latitude, longitude, altitudeGps );

        CayenneLppAddAnalogInput( channel++, AppGpsTime );
    }

    CayenneLppCopy( AppData.Buffer );
    AppData.BufferSize = CayenneLppGetSize( );

    if ( LmHandlerSend( &AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE ) == LORAMAC_HANDLER_SUCCESS )
    {
        GpioWrite( &LedBlue, 0 );
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
        default:
            // Intentional fall through
        case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
            break;
        case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
            break;
    }
}

static void UplinkProcess( void )
{
    static enum UplinkState
    {
        STATE_IDLE,
        STATE_WAIT_LOCATION,
        STATE_WAIT_JOIN,
    } state;
    static SysTime_t gpsTimeStart;

    uint8_t isTxPending = 0;
    uint8_t isGpsTimeout = 0;
    uint8_t isJoinComplete = 0;

    CRITICAL_SECTION_BEGIN( );
    isTxPending = IsTxFramePending;
    IsTxFramePending = 0;
    isGpsTimeout = IsGpsTimeout;
    IsGpsTimeout = 0;
    isJoinComplete = IsJoinComplete;
    IsJoinComplete = 0;
    CRITICAL_SECTION_END( );

    if( isTxPending == 1 && state == STATE_IDLE )
    {
        LpmSetStopMode( LPM_GPS_ID, LPM_DISABLE );
        printf( "Start GPS (timeout %d seconds)\r\n", GPS_TIMER_TIMEOUT / 1000 );
        GpsMcuIsDataParsed( ); // reset flag
        GpsStart( );
        TimerStart( &GpsTimer );
        gpsTimeStart = SysTimeGetMcuTime( );
        state = STATE_WAIT_LOCATION;
    }

    switch( state )
    {
    case STATE_IDLE:
        break;

    case STATE_WAIT_LOCATION:
        if( GpsMcuIsDataParsed( ) && GpsHasFix( ) )
        {
            SysTime_t gpsTimeEnd = SysTimeGetMcuTime( );
            AppGpsTime = ( gpsTimeEnd.Seconds - gpsTimeStart.Seconds );
            printf( "GPS has location %f sec\r\n", AppGpsTime );
        }
        else if( isGpsTimeout )
        {
            printf( "GPS time is out\r\n" );
        }
        else
        {
            break;
        }

        TimerStop( &GpsTimer );
        GpsStop( );
        LpmSetStopMode( LPM_GPS_ID, LPM_ENABLE );
        if( LmHandlerJoinStatus( ) != LORAMAC_HANDLER_SET )
        {
            printf( "Join to network...\r\n" );
            LmHandlerJoin();
            state = STATE_WAIT_JOIN;
        }
        else
        {
            printf( "Send frame\r\n" );
            PrepareTxFrame( );
            state = STATE_IDLE;
        }
        break;
    
    case STATE_WAIT_JOIN:
        if( isJoinComplete == 1 )
        {
            if( LmHandlerJoinStatus( ) != LORAMAC_HANDLER_SET )
            {
                printf( "Failed\r\n" );
            }
            else
            {
                printf( "Joined! Send frame\r\n" );
                PrepareTxFrame( );
            }
            state = STATE_IDLE;
        }
        break;
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
    }

    // Update timer periodicity
    TimerStop( &TxTimer );
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}
