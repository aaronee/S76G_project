/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board UART driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __UART_MCU_H__
#define __UART_MCU_H__

/*!
 * Enable UART2
 */
#define ENABLE_UART2            0
#define ENABLE_UART4            1

#ifdef ENABLE_UART4
#define DMA_BUFFER_SIZE         256
#define ENABLE_UART4_INTRRUPT   0
#define ENABLE_UART4_DMA        0
#define ENABLE_UART4_POLLING    1
#define LevelShifter_OE         PC_6

extern uint8_t DMA_UART4_BUFF[DMA_BUFFER_SIZE];
extern UART_HandleTypeDef Uart4Handle;
extern DMA_HandleTypeDef hdma_rx;
bool Uart4Init( void );
bool Uart4DeInit( void );
#endif

#ifdef ENABLE_UART2
extern UART_HandleTypeDef Uart2Handle;
bool Uart2Init( void );
bool Uart2DeInit( void );
#endif

typedef enum
{
    UART_1,
    UART_2,
    UART_USB_CDC = 255,
}UartId_t;

/*!
 * \brief Write formatted data to Uart and add line feed (new line)
 *
 * \param [IN] format  C string that contains a format string that follows the same
 *                     specifications as format in printf.
 * \param [IN] ...     Depending on the format string, the function may expect a
 *                     sequence of additional arguments, each containing a value
 *                     to be used to replace a format specifier in the format
 *                     string (or a pointer to a storage location, for n).
 */
void UartPrintLF(const char *format, ...);

/*!
 * \brief Write formatted data to Uart
 *
 * \param [IN] format  C string that contains a format string that follows the same
 *                     specifications as format in printf.
 * \param [IN] ...     Depending on the format string, the function may expect a
 *                     sequence of additional arguments, each containing a value
 *                     to be used to replace a format specifier in the format
 *                     string (or a pointer to a storage location, for n).
 */
void UartPrint(const char *format, ...);

/*!
 * \brief Write formatted data to Uart but not to trigger UART TX DMA
 *
 * \param [IN] format  C string that contains a format string that follows the same
 *                     specifications as format in printf.
 * \param [IN] ...     Depending on the format string, the function may expect a
 *                     sequence of additional arguments, each containing a value
 *                     to be used to replace a format specifier in the format
 *                     string (or a pointer to a storage location, for n).
 */
void UartPrintWait(const char *format, ...);

/*!
 * \brief To trigger UART TX DMA
 */
void UartPrintSend(void);

/*!
 * \brief Read formatted data from Uart
 *
 * \param [IN] buffer  C string that contains a sequence of characters
 *                     that control how characters extracted from the stream
 * \param [IN] size   wanted packet size
 */
uint32_t UartScan(uint8_t *buffer, uint32_t size);

/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 * \param [IN] tx   UART Tx pin name to be used
 * \param [IN] rx   UART Rx pin name to be used
 */
void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx );

/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj          UART object
 * \param [IN] mode         Mode of operation for the UART
 * \param [IN] baudrate     UART baudrate
 * \param [IN] wordLength   packet length
 * \param [IN] stopBits     stop bits setup
 * \param [IN] parity       packet parity
 * \param [IN] flowCtrl     UART flow control
 */
void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl );

/*!
 * \brief DeInitializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 */
void UartMcuDeInit( Uart_t *obj );

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Character to be sent
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data );

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Characters to be sent
 * \param [IN] size  number of characters to send
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *data, uint16_t size );

/*!
 * \brief Gets a character from the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Received character
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data );

/*!
 * \brief Gets a character from the UART (blocking mode)
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Received character
 * \param [IN] size  number of characters to be received
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *data, uint16_t size );

/*!
 * \brief Send a data buffer to UART (DMA mode)
 *
 * \retval status    [0: OK, 1: Failed]
 */
uint8_t UartMcuSendDma( void );

/*!
 * \brief Receive a data buffer from UART (IT mode)
 *
 * \retval status    [0: OK, 1: Failed]
 */
uint8_t UartMcuReceiveIT( void );

extern UART_HandleTypeDef UartHandle;

#endif // __UART_MCU_H__
