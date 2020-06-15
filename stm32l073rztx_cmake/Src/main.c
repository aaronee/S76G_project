#include "main.h"
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

extern int McuInitialized;

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

UART_HandleTypeDef huart1;
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );

/**
 * Peripheral Handler
 */
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void Radio_Init(void);
/**
 * Main application entry point.
 */
int main( void )
{

    bool isMaster = true;
    uint8_t i;

    // Target board initialization
    McuInitialized = false; //Give mcu init process for BoardInitMcu()
    BoardInitMcu( );
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    Radio_Init();

    while( 1 )
    {
        switch( State )
        {
            case RX:
                if( isMaster == true )
                {
                    if( BufferSize > 0 )
                    {
                        if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                        {
                            // Indicates on a LED that the received frame is a PONG
                            HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);

                            // Send the next PING frame
                            Buffer[0] = 'P';
                            Buffer[1] = 'I';
                            Buffer[2] = 'N';
                            Buffer[3] = 'G';
                            // We fill the buffer with numbers for the payload
                            for( i = 4; i < BufferSize; i++ )
                            {
                                Buffer[i] = i - 4;
                            }
                            DelayMs( 1 );
                            Radio.Send( Buffer, BufferSize );
                        }
                        else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                        { // A master already exists then become a slave
                            isMaster = false;
//                            GpioToggle( &Led2 ); // Set LED off
                            Radio.Rx( RX_TIMEOUT_VALUE );
                        }
                        else // valid reception but neither a PING or a PONG message
                        {    // Set device as master ans start again
                            isMaster = true;
                            Radio.Rx( RX_TIMEOUT_VALUE );
                        }
                    }
                }
                else
                {
                    if( BufferSize > 0 )
                    {
                        if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                        {
                            // Indicates on a LED that the received frame is a PING
                            HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);

                            // Send the reply to the PONG string
                            Buffer[0] = 'P';
                            Buffer[1] = 'O';
                            Buffer[2] = 'N';
                            Buffer[3] = 'G';
                            // We fill the buffer with numbers for the payload
                            for( i = 4; i < BufferSize; i++ )
                            {
                                Buffer[i] = i - 4;
                            }
                            DelayMs( 1 );
                            Radio.Send( Buffer, BufferSize );
                        }
                        else // valid reception but not a PING as expected
                        {    // Set device as master and start again
                            isMaster = true;
                            Radio.Rx( RX_TIMEOUT_VALUE );
                        }
                    }
                }
                State = LOWPOWER;
                break;
            case TX:
                // Indicates on a LED that we have sent a PING [Master]
                // Indicates on a LED that we have sent a PONG [Slave]
//                GpioToggle( &Led2 );
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                break;
            case RX_TIMEOUT:
            case RX_ERROR:
                if( isMaster == true )
                {
                    // Send the next PING frame
                    Buffer[0] = 'P';
                    Buffer[1] = 'I';
                    Buffer[2] = 'N';
                    Buffer[3] = 'G';
                    for( i = 4; i < BufferSize; i++ )
                    {
                        Buffer[i] = i - 4;
                    }
                    DelayMs( 1 );
                    Radio.Send( Buffer, BufferSize );
                }
                else
                {
                    Radio.Rx( RX_TIMEOUT_VALUE );
                }
                State = LOWPOWER;
                break;
            case TX_TIMEOUT:
                Radio.Rx( RX_TIMEOUT_VALUE );
                State = LOWPOWER;
                break;
            case LOWPOWER:
            default:
                // Set low power
                break;
        }

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    }

}


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LORA_radiosw_GPIO_Port, LORA_radiosw_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPS_rst_GPIO_Port, GPS_rst_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPS_vshifter_GPIO_Port, GPS_vshifter_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : SX_dio1_Pin */
    GPIO_InitStruct.Pin = SX_dio1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SX_dio1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LORA_radiosw_Pin */
    GPIO_InitStruct.Pin = LORA_radiosw_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_radiosw_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : GPS_rst_Pin */
    GPIO_InitStruct.Pin = GPS_rst_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPS_rst_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SX_dio0_Pin SX_dio4_Pin SX_dio3_Pin GPS_1PPS_Pin
                             SX_dio2_Pin */
    GPIO_InitStruct.Pin = SX_dio0_Pin | SX_dio4_Pin | SX_dio3_Pin | GPS_1PPS_Pin
                          | SX_dio2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : GPS_vshifter_Pin */
    GPIO_InitStruct.Pin = GPS_vshifter_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPS_vshifter_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SX_dio5_Pin */
    GPIO_InitStruct.Pin = SX_dio5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SX_dio5_GPIO_Port, &GPIO_InitStruct);

    //Configure GPIO pin : LED IO PB7
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Radio_Init(void)
{
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );

    Radio.Rx( RX_TIMEOUT_VALUE );
}
// LORA event callback functions
void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}
// EO LORA event callback functions

void Error_Handler(void)
{
    while (1);
}