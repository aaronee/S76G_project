/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Implements the generic UART driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "uart-board.h"
#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

#include "uart.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10

/*!
 * UartPrint sub-function print internal define
 */
#define PAD_RIGHT        1
#define PAD_ZERO         2
#define PRINT_BUF_LEN    12 // the following should be enough for 32 bit int

/*!
 * UartPrint variables
 */
static uint8_t out[UART_LENGTH];
static uint32_t value1;
static uint8_t **pout1 = (uint8_t**)&value1;

static uint32_t value2;
static uint8_t **pout2 = (uint8_t**)&value2;

static uint32_t value3;
static uint8_t **pout3 = (uint8_t**)&value3;

////// static functions //////

/*!
 * UartPrint sub-functions
 */
static void printchar(char **str, int c)
{
    //extern int putchar(int c);
    if (str) {
        **str = c;
        ++(*str);
    }
    else (void)putchar(c);
}

/*!
 * UartPrint sub-functions
 */
static int prints(char **out, const char *string, int width, int pad)
{
    register int pc = 0, padchar = ' ';

    if (width > 0) {
        register int len = 0;
        register const char *ptr;
        for (ptr = string; *ptr; ++ptr) ++len;
        if (len >= width) width = 0;
        else width -= len;
        if (pad & PAD_ZERO) padchar = '0';
    }
    if (!(pad & PAD_RIGHT)) {
        for ( ; width > 0; --width) {
            printchar (out, padchar);
            ++pc;
        }
    }
    for ( ; *string ; ++string) {
        printchar (out, *string);
        ++pc;
    }
    for ( ; width > 0; --width) {
        printchar (out, padchar);
        ++pc;
    }

    return pc;
}

/*!
 * UartPrint sub-functions
 */
static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
    char print_buf[PRINT_BUF_LEN];
    register char *s;
    register int t, neg = 0, pc = 0;
    register unsigned int u = i;

    if (i == 0) {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints (out, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0) {
        neg = 1;
        u = -i;
    }

    s = print_buf + PRINT_BUF_LEN-1;
    *s = '\0';

    while (u) {
        t = u % b;
        if( t >= 10 )
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }

    if (neg) {
        if( width && (pad & PAD_ZERO) ) {
            printchar (out, '-');
            ++pc;
            --width;
        }
        else {
            *--s = '-';
        }
    }

    return pc + prints (out, s, width, pad);
}

/*!
 * UartPrint sub-functions
 */
static int print(char **out, int *varg)
{
    register int width, pad;
    register int pc = 0;
    register char *format = (char *)(*varg++);
    char scr[2];

    for (; *format != 0; ++format) {
        if (*format == '%') {
            ++format;
            width = pad = 0;
            if (*format == '\0') break;
            if (*format == '%') goto out;
            if (*format == '-') {
                ++format;
                pad = PAD_RIGHT;
            }
            while (*format == '0') {
                ++format;
                pad |= PAD_ZERO;
            }
            for ( ; *format >= '0' && *format <= '9'; ++format) {
                width *= 10;
                width += *format - '0';
            }
            if( *format == 's' ) {
                register char *s = *((char **)varg++);
                pc += prints (out, s?s:"(null)", width, pad);
                continue;
            }
            if( *format == 'd' ) {
                pc += printi (out, *varg++, 10, 1, width, pad, 'a');
                continue;
            }
            if( *format == 'x' ) {
                pc += printi (out, *varg++, 16, 0, width, pad, 'a');
                continue;
            }
            if( *format == 'X' ) {
                pc += printi (out, *varg++, 16, 0, width, pad, 'A');
                continue;
            }
            if( *format == 'u' ) {
                pc += printi (out, *varg++, 10, 0, width, pad, 'a');
                continue;
            }
            if( *format == 'c' ) {
                /* char are converted to int then pushed on the stack */
                scr[0] = *varg++;
                scr[1] = '\0';
                pc += prints (out, scr, width, pad);
                continue;
            }
        }
        else {
        out:
            printchar (out, *format);
            ++pc;
        }
    }
    if (out) **out = '\0';
    return pc;
}

/*!
 * If last byte of a string is not "NL", add "NL"
 */
static void append_new_line(char* out)
{
    size_t len;
    len = strlen(out);
    if (len != 0 && (out[len-1] != 0xA)) {
        strcat(out, "\n");
    }
}

////// non-static functions //////
void UartPrintLF(const char *format, ...)
{
#if UART_ENABLE
    *pout1 = &out[0];

    memset(out, 0x00, UART_LENGTH);
    register int *varg = (int *)(&format);

    //Copy strings into character array
    print((char**)pout1, varg);

    //Add a LF char at the end of "out" if no LF
    append_new_line((char*)out);

    //Store into UART Tx Buffer
    UartPutBuffer(&Uart, out, strlen((const char*)out));

    //Trigger UART TX DMA
    UartMcuSendDma();
#else
    UNUSED(format);
#endif
}

void UartPrint(const char *format, ...)
{
#if UART_ENABLE
    *pout2 = &out[0];

    memset(out, 0x00, UART_LENGTH);
    register int *varg = (int *)(&format);

    //Copy strings into character array
    print((char**)pout2, varg);

    //Store into UART Tx Buffer
    UartPutBuffer(&Uart, out, strlen((const char*)out));

    //Trigger UART TX DMA
    UartMcuSendDma();
#else
    UNUSED(format);
#endif
}

void UartPrintWait(const char *format, ...)
{
#if UART_ENABLE
    *pout3 = &out[0];

    memset(out, 0x00, UART_LENGTH);
    register int *varg = (int *)(&format);

    //Copy strings into character array
    print((char**)pout3, varg);

    //Store into UART Tx Buffer
    UartPutBuffer(&Uart, out, strlen((const char*)out));
#else
    UNUSED(format);
#endif
}

void UartPrintSend(void)
{
    //Trigger UART TX DMA
    UartMcuSendDma();
}

uint32_t UartScan(uint8_t *buffer, uint32_t size)
{
#if UART_ENABLE
    uint32_t readBytes = 0;

    //Clear UART buffer for Rx
    memset(buffer, 0x00, size);

    //Read n Bytes from UART Rx Fifo
    UartGetBuffer(&Uart, buffer, size, (uint16_t*)&readBytes);

    return readBytes;
#else
    UNUSED(buffer);
#endif
}

void UartInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx )
{
    if( obj->IsInitialized == false )
    {
        obj->IsInitialized = true;

        if( uartId == UART_USB_CDC )
        {
#if defined( USE_USB_CDC )
            UartUsbInit( obj, uartId, NC, NC );
#endif
        }
        else
        {
            UartMcuInit( obj, uartId, tx, rx );
        }
    }
}

void UartConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    if( obj->IsInitialized == false )
    {
        // UartInit function must be called first.
        assert_param( FAIL );
    }
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbConfig( obj, mode, baudrate, wordLength, stopBits, parity, flowCtrl );
#endif
    }
    else
    {
        UartMcuConfig( obj, mode, baudrate, wordLength, stopBits, parity, flowCtrl );
    }
}

void UartDeInit( Uart_t *obj )
{
    obj->IsInitialized = false;
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        UartUsbDeInit( obj );
#endif
    }
    else
    {
        UartMcuDeInit( obj );
    }
}

uint8_t UartPutChar( Uart_t *obj, uint8_t data )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbPutChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        return UartMcuPutChar( obj, data );
    }
}

uint8_t UartGetChar( Uart_t *obj, uint8_t *data )
{
    if( obj->UartId == UART_USB_CDC )
    {
#if defined( USE_USB_CDC )
        return UartUsbGetChar( obj, data );
#else
        return 255; // Not supported
#endif
    }
    else
    {
        return UartMcuGetChar( obj, data );
    }
}

uint8_t UartPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size )
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

uint8_t UartGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes )
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

