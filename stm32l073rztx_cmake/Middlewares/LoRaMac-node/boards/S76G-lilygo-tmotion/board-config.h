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


#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

#ifndef debug_printf
#define debug_printf UartPrintLF
#endif

#define DO_NOTHING()   do {} while(0)

/*!
 * Enables the choice between Led1 and Potentiometer.
 * LED1 and Potentiometer are exclusive.
 * \remark When using Potentimeter don't forget  that the connection between
 *         ADC input pin of iM880A and the Demoboard Poti requires a connection
 *         between X5:11 - X5:18.
 *         Remove the original jumpers for that. 
 *         On SK-iM880A X5 is the 20 pin header close to the DIP SW and Buttons
 */

/*!
 * UART Baud Rate Setting
 */
#define UART_BAUDRATE 115200

/*!
 * UART Tx Rx Buffer
 */
#define UART_LENGTH 540 // ( 250*2 + 40 )

/*!
 * UART Rx Buffer
 */
#define UART_RX_DMA_LENGTH   1
#define UART_RX_LOCAL_LENGTH (128 + 1) //One additional bytes for filling NULL character
#define UART_RX_FIFO_LENGTH  1024

/*!
 * Enable Power Saving Demo or Not
 */
#define ENABLE_POWER_SAVING        0  // 1:Enable, 0:Disable
#define POWER_SAVING_GPS_Sleep0    1  // 1:Enable, 0:Disable
#define POWER_SAVING_GPS_Sleep1    0  // 1:Enable, 0:Disable
#define POWER_SAVING_GPS_Sleep2    0  // 1:Enable, 0:Disable
/*!
 * Set Power Saving Interval Time
 */
// #define POWER_SAVING_INTERVAL  30  // second

 /*!
  * Enable I2C
  */
//#define ENABLE_I2C  

/*!
 * Battery Level is read by UART command or not
 */
#define BATT_LEVEL_BY_CMD      1  // 1:Enable, 0:Disable

/*!
 * Board MCU pins definitions
 */

//LevelShifter_OEpin
#define LevelShifter_OEpin							PC_6

//Disable Ant Switch GPIOs
#define RADIO_ANT_SWITCH_RXTX                       PA_1 //1:Rx, 0:Tx
	
#define RADIO_RESET                                 PB_10 //SPI2
#define RADIO_MOSI                                  PB_15 //SPI2
#define RADIO_MISO                                  PB_14 //SPI2
#define RADIO_SCLK                                  PB_13 //SPI2
#define RADIO_NSS                                   PB_12 //SPI2

#define RADIO_DIO_0                                 PB_11
#define RADIO_DIO_1                                 PC_13
#define RADIO_DIO_2                                 PB_9
#define RADIO_DIO_3                                 PB_4
#define RADIO_DIO_4                                 PB_3
#define RADIO_DIO_5                                 PA_15


#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15
#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1


#define LoRa_TCXO_OE_Pin                            PD_7 //TCXO enable pin for SX1276
#define LoRa_OSC_SEL_Pin                            PC_1 //TCXO sellect pin - input - LOW-TXC0/HIGH-internal_crystal	

//SWD pin on STM32L073RZ
#define SWCLK                                       PA_14
#define SWDAT                                       PA_13

//I2C on board lilygo Tmotion board
#define I2C_SCL                                     PB_6 //I2C1 SCL
#define I2C_SDA                                     PB_7 //I2C1 SDA

//UART on board lilygo Tmotion board
#define UART_TX                                     PA_9  //USART1 TX
#define UART_RX                                     PA_10 //USART1 RX

//Enable in S76G project
#define GPS_RST_X                                   PB_2

//Disable USB D+ D-
#if ACSIP_DEFAULT_DISABLE
#define USB_DM                                      PA_11
#define USB_DP                                      PA_12
#endif

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL                       DMA1_Channel2
#define USARTx_RX_DMA_CHANNEL                       DMA1_Channel3


#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
