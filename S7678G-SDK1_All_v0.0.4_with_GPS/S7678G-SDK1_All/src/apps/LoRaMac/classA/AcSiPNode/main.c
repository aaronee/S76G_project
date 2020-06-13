/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Andreas Pella (IMST GmbH), Miguel Luis and Gregory Cristian
*/

/*! \file classA and C main.c */

#include <string.h>
#include <math.h>
#include "board.h"

#include "LoRaMac.h"
#include "Region.h"
#include "Commissioning.h"

void prepare_palyload_data(uint8_t *);
void print_GPS_coordinate(uint8_t *);

#define DIVIDE_INTO_SECONDS                         1 //units of longitude and latitude would show second
#define DISPLAY_DATA_UTC                            1 //to display the data and time from GPS
#define GPS_1PPS_Output                             1 //used to control 1PPS output

/*!
 * 
 */
#if DIVIDE_INTO_SECONDS
#define BUFFER_SIZE                                 26 //It is only for RMC demo payload length
#else
#define BUFFER_SIZE                                 24 //It is only for RMC demo payload length
#endif

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#if GPS_ActiveLowPowerControlMode == 0
#define APP_TX_DUTYCYCLE                            10000
#else
#define APP_TX_DUTYCYCLE                            GPS_ActiveLowPowerMode_DUTYCYCLE
#endif

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_3

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          0

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            3

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_SIZE                       BUFFER_SIZE

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED4
 */
static TimerEvent_t Led4Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

//GPS API instance
GPS_Engine S76G_cxd5603;

//NMEA structures
RMC_Data RMC;
GGA_Data GGA;
GLL_Data GLL;
GNS_Data GNS;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 3:
        {
            /*
            uint8_t potiPercentage = 0;
            uint16_t vdd = 0;

            // Read the current potentiometer setting in percent
            potiPercentage = BoardMeasurePotiLevel( );

            // Read the current voltage level
            BoardGetBatteryLevel( ); // Updates the value returned by BoardGetBatteryVoltage( ) function.

            #if 0
            vdd = BoardGetBatteryVoltage( );
            #else
            vdd = 78;
            #endif

            AppData[0] = AppLedStateOn;
            AppData[1] = potiPercentage;
            AppData[2] = ( vdd >> 8 ) & 0xFF;
            AppData[3] = vdd & 0xFF;
            */

            AppData[0] = 'G';
            AppData[1] = 'P';
            AppData[2] = 'S';
            AppData[3] = ':';
            // Fill the latitude and longitude data to paylod
            prepare_palyload_data(AppData);
            // Print received payload by UART
            print_GPS_coordinate(AppData);
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void )
{
    TimerStop( &Led4Timer );
    // Switch LED 4 OFF
    GpioWrite( &Led4, 0 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    GpioWrite( &Led2, 0 );
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 4 ON
        GpioWrite( &Led4, 1 );
        TimerStart( &Led4Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            //AcSiP(+), for printing data when received downlink data
            UartPrintWait("\n\r>> mac rx %d ", mcpsIndication->Port);
            for(uint8_t i = 0 ; i < mcpsIndication->BufferSize ; i++) {
                UartPrintWait("%02x", mcpsIndication->Buffer[i]);
            }
            UartPrintWait("\n\r");
            UartPrintSend();

            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite( &Led2, 1 );
    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;

                //AcSiP(+)
                debug_printf( "\n\r>> accepted" );
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );

#if ENABLE_UART4
    Uart4Init();
    DelayMs( 50 );
#endif

    //GPS Engine init
    S76G_cxd5603.State = GpsState_Init;
    S76G_cxd5603.Command = CmdNum_None;
    S76G_cxd5603.WordCount = 0;
    S76G_cxd5603.GPGGA = &GGA;
    S76G_cxd5603.GPRMC = &RMC;
    S76G_cxd5603.GPGLL = &GLL;
    S76G_cxd5603.GPGNS = &GNS;

#if GPS_ActiveLowPowerControlMode == 0
    //GPS initialization
    //Default setting :
    //1. Seatellite : GPS & GLONAS (Hybrid)
    //2. NMEA : ALL
    //3. Time interval : APP_TX_DUTYCYCLE ms
    GpsMainInit(&S76G_cxd5603, GPS_BSSL_RMC, APP_TX_DUTYCYCLE); //RMC, APP_TX_DUTYCYCLE ms interval
    //GpsMainInit(&S76G_cxd5603, GPS_BSSL_ALL, APP_TX_DUTYCYCLE); //All NMEA, APP_TX_DUTYCYCLE ms interval
    //GpsMainInit(&S76G_cxd5603, GPS_BSSL_GGA, 1000); //GGA, 1 second interval
    //GpsMainInit(&S76G_cxd5603, GPS_BSSL_GLL, 1000); //GGL, 1 second interval
    //GpsMainInit(&S76G_cxd5603, GPS_BSSL_GNS, 1000); //GGL, 1 second interval
#else
    GpsMainInit(&S76G_cxd5603, GPS_BSSL_ALL, GPS_ActiveLowPowerMode_DUTYCYCLE); //RMC, APP_TX_DUTYCYCLE ms interval
#endif

    UartPrint( "\nS7678G LoRaWAN with GPS SDK v0.0.4\n\n" );

//AcSiP(+), For demostrate power saving
#if ENABLE_POWER_SAVING
    //1. @GSTP
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Idle, 0);
    //2. @BUP
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_BackupData, 0);

    //3. @SLP
#if POWER_SAVING_GPS_Sleep0 == 1
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Sleep, 0);  //for GPS sleep level 0
    UartPrint( "GPS Sleep Level 0\n");
#elif POWER_SAVING_GPS_Sleep1 == 1
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Sleep, 1);    //for GPS sleep level 1
    UartPrint( "GPS Sleep Level 1\n");
#elif POWER_SAVING_GPS_Sleep2 == 1
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Sleep, 2);    //for GPS sleep level 2
    UartPrint( "GPS Sleep Level 2\n");
#else
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Sleep, 0);  //for GPS sleep level 0
    UartPrint( "GPS Sleep Level 0\n");
#endif
    DelayMs( 10 );

    //Enter power-saving stop mode and then back to normal mode.
    UartPrint( "Enter Power Saving mode for %ds ...\n", POWER_SAVING_INTERVAL );
    DelayMs( 10 );
    PowerSaving( POWER_SAVING_INTERVAL, GPIOC );
    DelayMs( 10 );
    UartPrint( "Leave Power Saving mode ...\n", POWER_SAVING_INTERVAL );

#if ENABLE_UART4
    Uart4Init();
    DelayMs( 50 );
#endif

    //4. @WUP -- Run twice to make sure GPS wakeup
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Wake, 0);
    // Delay for waiting GPS restores the data in RAM from Flash
    DelayMs( 1000 );
    //5. Hot start
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_HotStart, 0);
#endif

#if GPS_1PPS_Output
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_Idle, 0);
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_1PPS, 1);
    // 1:Enable 1PPS output, 0:Disable 1PPS output
    GpsCommand_block_until_done(&S76G_cxd5603, CmdNum_HotStart, 0);
#endif

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        gps_collect();

        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
#if defined( REGION_AS923 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN779 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_CN470 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_EU433 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
#elif defined( REGION_EU868 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_IN865 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_KR920 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_US915_HYBRID )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID );
#else
    #error "Please define a region in the compiler options."
#endif
                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit( &Led4Timer, OnLed4TimerEvent );
                TimerSetValue( &Led4Timer, 25 );

                TimerInit( &Led2Timer, OnLed2TimerEvent );
                TimerSetValue( &Led2Timer, 25 );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
                LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

                mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
                mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_RX2_CHANNEL;
                mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif

#ifdef Class_C
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_C;
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                //AcSiP(-), DevEui value is set by define currently
                //BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    srand1( BoardGetRandomSeed( ) );

                    // Choose a random device address
                    DevAddr = randr( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}

#if DIVIDE_INTO_SECONDS
//Payload data format
#define OFFSET_LatitudeNS         4 
#define OFFSET_LatitudeDegree     5
#define OFFSET_LatitudeMinute     6
#define OFFSET_LatitudeSecond     7
#define OFFSET_LongitudaEW       11
#define OFFSET_LongitudaDegree   12
#define OFFSET_LongitudaMinute   13
#define OFFSET_LongitudaSecond   14
#define OFFSET_Speed             18
#define OFFSET_UTC_Year          20
#define OFFSET_UTC_Month         21
#define OFFSET_UTC_Day           22
#define OFFSET_UTC_Hour          23
#define OFFSET_UTC_Min           24
#define OFFSET_UTC_Sec           25
#else
//Payload data format
#define OFFSET_LatitudeNS         4 
#define OFFSET_LatitudeDegree     5
#define OFFSET_LatitudeMinute     6`
#define OFFSET_LongitudaEW       10
#define OFFSET_LongitudaDegree   11
#define OFFSET_LongitudaMinute   12
#define OFFSET_Speed             16
#define OFFSET_UTC_Year          18
#define OFFSET_UTC_Month         19
#define OFFSET_UTC_Day           20
#define OFFSET_UTC_Hour          21
#define OFFSET_UTC_Min           22
#define OFFSET_UTC_Sec           23
#endif

void prepare_palyload_data(uint8_t *buf)
{
#if DIVIDE_INTO_SECONDS
    uint8_t Lat_min = 0, Long_min = 0;
    uint32_t Lat_sec = 0, Long_sec = 0;

    Lat_min = (uint8_t)(S76G_cxd5603.GPRMC->LatitudeMinute / 10000);
    Long_min = (uint8_t)(S76G_cxd5603.GPRMC->LongitudaMinute / 10000);

    Lat_sec = (uint32_t)(S76G_cxd5603.GPRMC->LatitudeMinute % 10000) * 60;
    Long_sec = (uint32_t)(S76G_cxd5603.GPRMC->LongitudaMinute % 10000) * 60;
#endif
    buf[OFFSET_LatitudeNS] = (uint8_t)S76G_cxd5603.GPRMC->LatitudeNS;
    buf[OFFSET_LatitudeDegree] = (uint8_t)S76G_cxd5603.GPRMC->LatitudeDegree;
#if DIVIDE_INTO_SECONDS
    buf[OFFSET_LatitudeMinute] = Lat_min;
    memcpy(buf+OFFSET_LatitudeSecond, (unsigned char *)&Lat_sec, sizeof(uint32_t));
#else
    memcpy(buf+OFFSET_LatitudeMinute, (unsigned char *)&S76G_cxd5603.GPRMC->LatitudeMinute, sizeof(uint32_t));
#endif

    buf[OFFSET_LongitudaEW] = (uint8_t)S76G_cxd5603.GPRMC->LongitudaEW;
    buf[OFFSET_LongitudaDegree] = (uint8_t)S76G_cxd5603.GPRMC->LongitudaDegree;
#if DIVIDE_INTO_SECONDS
    buf[OFFSET_LongitudaMinute] = Long_min;
    memcpy(buf+OFFSET_LongitudaSecond, (unsigned char *)&Long_sec, sizeof(uint32_t));
#else
    memcpy(buf+OFFSET_LongitudaMinute, (unsigned char *)&S76G_cxd5603.GPRMC->LongitudaMinute, sizeof(uint32_t));
#endif
    memcpy(buf+OFFSET_Speed, (unsigned char *)&S76G_cxd5603.GPRMC->Speed, sizeof(uint16_t));

    //buf[OFFSET_LongitudaEW] = (uint8_t)S76G_cxd5603.GPRMC->LongitudaEW;
    buf[OFFSET_UTC_Year] = S76G_cxd5603.GPRMC->Date[2];
    buf[OFFSET_UTC_Month] = S76G_cxd5603.GPRMC->Date[1];
    buf[OFFSET_UTC_Day] = S76G_cxd5603.GPRMC->Date[0];
    buf[OFFSET_UTC_Hour] = S76G_cxd5603.GPRMC->UTC[0];
    buf[OFFSET_UTC_Min] = S76G_cxd5603.GPRMC->UTC[1];
    buf[OFFSET_UTC_Sec] = S76G_cxd5603.GPRMC->UTC[2];
}

void print_GPS_coordinate(uint8_t *buf)
{
#if DIVIDE_INTO_SECONDS
    int i;
    uint32_t Lat_sec = 0, Long_sec = 0;
    uint16_t Speed;

    //Binary array to integer
    for (i = 3 ; i >= 0 ; i--) {
        Lat_sec <<= 8;
        Lat_sec |= buf[OFFSET_LatitudeSecond+i];
    }

    for (i = 3 ; i >= 0 ; i--)  {
        Long_sec <<= 8;
        Long_sec |= buf[OFFSET_LongitudaSecond+i];
    }

    for (i = 1 ; i >= 0 ; i--) {
        Speed <<= 8;
        Speed |= buf[OFFSET_Speed+i];
    }

#if DISPLAY_DATA_UTC
    uint8_t year, month, day;
    uint8_t hour, min, sec, millisec;
    year = S76G_cxd5603.GPRMC->Date[2];
    month = S76G_cxd5603.GPRMC->Date[1];
    day = S76G_cxd5603.GPRMC->Date[0];
    hour = S76G_cxd5603.GPRMC->UTC[0];
    min = S76G_cxd5603.GPRMC->UTC[1];
    sec = S76G_cxd5603.GPRMC->UTC[2];
    millisec = S76G_cxd5603.GPRMC->UTC[3];
    UartPrint("\n\rS7XG Data and Time >> %d-%02d-%02d %02d:%02d:%02d.%02d\n", 
             (year+2000) , month, day, hour, min, sec, millisec);
#endif

    UartPrint("S7XG Coordinate >> Lat. & Long. : %c %d %d %d.%d %c %d %d %d.%d ,Speed %d.%d knot\n", 
             (buf[OFFSET_LatitudeNS]?'N':'S'), buf[OFFSET_LatitudeDegree], buf[OFFSET_LatitudeMinute], Lat_sec/10000, Lat_sec%10000, (buf[OFFSET_LongitudaEW]?'E':'W'), 
             buf[OFFSET_LongitudaDegree], buf[OFFSET_LongitudaMinute], Long_sec/10000, Long_sec%10000, Speed/10, Speed%10);
#else
    int i;
    uint32_t Lat_min = 0, Long_min = 0;
    uint16_t Speed;

    //Binary array to integer
    for (i = 3 ; i >= 0 ; i--) {
        Lat_min <<= 8;
        Lat_min |= buf[OFFSET_LatitudeMinute+i];
    }

    for (i = 3 ; i >= 0 ; i--) {
        Long_min <<= 8;
        Long_min |= buf[OFFSET_LongitudaMinute+i];
   }

    for (i = 1 ; i >= 0 ; i--) {
        Speed <<= 8;
        Speed |= buf[OFFSET_Speed+i];
   }

    UartPrint("S7XG Coordinate >> Lat. & Long. : %c %d %d.%d %c %d %d.%d ,Speed %d.%d knot\n", 
             (buf[OFFSET_LatitudeNS]?'N':'S'), buf[OFFSET_LatitudeDegree], Lat_min/10000, Lat_min%10000, (buf[OFFSET_LongitudaEW]?'E':'W'), 
             buf[OFFSET_LongitudaDegree], Long_min/10000, Long_min%10000, Speed/10, Speed%10);
#endif
}
