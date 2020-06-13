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
#include "board.h"
#include "uart-board.h"

/*!
 * Internal handle in uart-board.c for AcSiP UART command transmission
 */
UART_HandleTypeDef UartHandle;
#if ( ENABLE_UART2 )
UART_HandleTypeDef Uart2Handle;
#endif
#if ( ENABLE_UART4 )
UART_HandleTypeDef Uart4Handle;
#ifdef LevelShifter_OEpin
static Gpio_t LS_OE;
#endif
#endif

/*!
 * One byte data for UART RX buffer
 */
uint8_t RxData = 0;
#if ( ENABLE_UART2 )
uint8_t RxData2 = 0;
#endif
#if ( ENABLE_UART4 )
uint8_t DMA_UART4_BUFF[DMA_BUFFER_SIZE];
#endif

/*!
 * TX/RX complete callback flags
 */
__IO bool UartTxReady = 1;
__IO bool UartRxReady = 1;

/*!
 * TX/RX temporary buffer between functions
 */
static uint8_t tx_buf[UART_LENGTH];
static uint8_t rx_buf[1];

#if ENABLE_UART4
DMA_HandleTypeDef hdma_rx;
#endif

void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    __HAL_RCC_USART1_FORCE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_ENABLE( );

    __HAL_RCC_DMA1_CLK_ENABLE();

    GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF4_USART1 );
    GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF4_USART1 );
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    static DMA_HandleTypeDef hdma_tx;

    UartHandle.Instance = USART1;
    UartHandle.Init.BaudRate = baudrate;

    if( mode == TX_ONLY )
    {
        if( obj->FifoTx.Data == NULL )
        {
            assert_param( FAIL );
        }
        UartHandle.Init.Mode = UART_MODE_TX;
    }
    else if( mode == RX_ONLY )
    {
        if( obj->FifoRx.Data == NULL )
        {
            assert_param( FAIL );
        }
        UartHandle.Init.Mode = UART_MODE_RX;
    }
    else if( mode == RX_TX )
    {
        if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
        {
            assert_param( FAIL );
        }
        UartHandle.Init.Mode = UART_MODE_TX_RX;
    }
    else
    {
       assert_param( FAIL );
    }

    if( wordLength == UART_8_BIT )
    {
        UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else if( wordLength == UART_9_BIT )
    {
        UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
    }

    switch( stopBits )
    {
    case UART_2_STOP_BIT:
        UartHandle.Init.StopBits = UART_STOPBITS_2;
        break;
    case UART_1_STOP_BIT:
    default:
        UartHandle.Init.StopBits = UART_STOPBITS_1;
        break;
    }

    if( parity == NO_PARITY )
    {
        UartHandle.Init.Parity = UART_PARITY_NONE;
    }
    else if( parity == EVEN_PARITY )
    {
        UartHandle.Init.Parity = UART_PARITY_EVEN;
    }
    else
    {
        UartHandle.Init.Parity = UART_PARITY_ODD;
    }

    if( flowCtrl == NO_FLOW_CTRL )
    {
        UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }
    else if( flowCtrl == RTS_FLOW_CTRL )
    {
        UartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS;
    }
    else if( flowCtrl == CTS_FLOW_CTRL )
    {
        UartHandle.Init.HwFlowCtl = UART_HWCONTROL_CTS;
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL )
    {
        UartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }

    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    if( HAL_UART_Init( &UartHandle ) != HAL_OK )
    {
        while( 1 );
    }

    /*###### Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tx.Init.Request             = DMA_REQUEST_3;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

    /*###### Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USART1_TX) */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    HAL_NVIC_SetPriority( USART1_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( USART1_IRQn );

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
    rx_buf[0] = 0x00;
    HAL_UART_Receive_IT( &UartHandle, rx_buf, 1 );
}

void UartMcuDeInit( Uart_t *obj )
{
    __HAL_RCC_USART1_FORCE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_DISABLE( );

    GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    uint32_t prim;
    if( IsFifoFull( &obj->FifoTx ) == false )
    {
        prim = __get_PRIMASK();
        __disable_irq( );
        FifoPush( &obj->FifoTx, data );
        if(!prim)
            __enable_irq( );

        return 0; // OK
    }
    return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    uint32_t prim;
    if( IsFifoEmpty( &obj->FifoRx ) == false )
    {
        prim = __get_PRIMASK();
        __disable_irq( );
        *data = FifoPop( &obj->FifoRx );
        if(!prim)
            __enable_irq( );
        return 0;
    }
    return 1;
}

uint8_t UartMcuSendDma( void )
{
    uint8_t data = 0;
    uint32_t len = 0;
    uint32_t prim = 0;

    if (UartTxReady == 1) {
        //Clear local memory Tx buffer
        memset(tx_buf, 0x00, UART_LENGTH);

        //Fetch data from tx fifo to local memory
        while ( IsFifoEmpty( &Uart.FifoTx ) == false ) {
            prim = __get_PRIMASK();
            __disable_irq( );
            data = FifoPop( &Uart.FifoTx );
            if(!prim)
                __enable_irq( );
            tx_buf[len++] = data;
        }
        //Launch UART Tx DMA if data is ready to be sent.
        if (len > 0) {
            //if( HAL_UART_Transmit_DMA(&UartHandle, aTxBuffer, TXBUFFERSIZE) != HAL_OK )
            if( HAL_UART_Transmit_DMA(&UartHandle, tx_buf, len) != HAL_OK )
                return 1;
            //Uart Tx DMA launched, Set flag as "busy"
            UartTxReady = 0;

            //AcSiP(+), Trigger UART RX IT mode if it has RX data
            HAL_UART_Receive_IT(&UartHandle, rx_buf, 1 );
        }
    }
    return 0;
}

uint8_t UartMcuReceiveIT( void )
{
    uint32_t prim = 0;

    if ( UartRxReady == 1 ) {
        //Check if needs to stop fifo pushing if 0x00 meets
        if ( rx_buf[0] != 0x00 ) {
            //Save one byte data from rx buffer into rx fifo
            prim = __get_PRIMASK();
            __disable_irq( );
            if ( IsFifoFull( &Uart.FifoRx ) == false ) {
                FifoPush( &Uart.FifoRx, rx_buf[0] );
            }
            if(!prim)
                __enable_irq( );
        }

        if ( HAL_UART_Receive_IT(&UartHandle, rx_buf, 1 ) != HAL_OK )
            return 1;
    }
    return 0;
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *UartHandle )
{
    UartTxReady = 1;
    //Send UART Tx Data if Tx Fifo has data.
    UartMcuSendDma();

    if( Uart.IrqNotify != NULL )
    {
        Uart.IrqNotify( UART_NOTIFY_TX );
    }
}


#if ENABLE_UART4_DMA
void HAL_UART_RxHalfCpltCallback( UART_HandleTypeDef *UartHandle )
{
    if(UartHandle->Instance==USART4)
    {
        //int DMA_Ptr;
        //DMA_Ptr = DMA_BUFFER_SIZE - hdma_rx.Instance->CNDTR;
        HAL_UART_Transmit_DMA(&Uart4Handle, DMA_UART4_BUFF, DMA_BUFFER_SIZE/2);
    }
}
#endif

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *UartHandle )
{
#if ENABLE_UART4_DMA
    if(UartHandle->Instance==USART4)
    {
        //int DMA_Ptr;
        //DMA_Ptr = DMA_BUFFER_SIZE - hdma_rx.Instance->CNDTR;
        HAL_UART_Transmit_DMA(&Uart4Handle, DMA_UART4_BUFF+(DMA_BUFFER_SIZE/2), DMA_BUFFER_SIZE/2);
    }
#endif
#if ENABLE_UART4_INTRRUPT
    if(UartHandle->Instance==USART4)
    {
        while(HAL_UART_Transmit_IT(&Uart2Handle, &RxData4, 1) != HAL_OK);
        while(HAL_UART_Receive_IT(&Uart4Handle, &RxData4, 1) != HAL_OK);
    }
#endif
#if ENABLE_UART2
    if(UartHandle->Instance==USART2)
    {
        while(HAL_UART_Transmit_IT(&Uart2Handle, &RxData2, 1) != HAL_OK);
        while(HAL_UART_Receive_IT(&Uart2Handle, &RxData2, 1) != HAL_OK);
    }
#endif
    if(UartHandle->Instance==USART1)
    {
        UartRxReady = 1;
        UartMcuReceiveIT();
    }
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *UartHandle )
{
    __HAL_UART_CLEAR_OREFLAG(UartHandle);
    __HAL_UART_CLEAR_NEFLAG(UartHandle);
    __HAL_UART_CLEAR_FEFLAG(UartHandle);
}

void USART1_IRQHandler( void )
{
    HAL_UART_IRQHandler( &UartHandle );
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
#if ENABLE_UART4
    static DMA_HandleTypeDef hdma_tx;
#endif
    GPIO_InitTypeDef GPIO_InitStruct;
#if ENABLE_UART2
    if(huart->Instance==USART2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
#endif
#if ENABLE_UART4
    if(huart->Instance==USART4)
    {
        __HAL_RCC_USART4_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_USART4;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

#ifdef LevelShifter_OEpin
        GpioInit( &LS_OE, LevelShifter_OE, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioWrite( &LS_OE, 1 );
#endif

#if ENABLE_UART4_INTRRUPT
        HAL_NVIC_SetPriority(USART4_5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART4_5_IRQn);
#else
        hdma_tx.Instance                 = DMA1_Channel7;
        hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx.Init.Mode                = DMA_NORMAL;
        hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_tx.Init.Request             = DMA_REQUEST_12;
        HAL_DMA_Init(&hdma_tx);
        __HAL_LINKDMA(huart, hdmatx, hdma_tx);

        hdma_rx.Instance                 = DMA1_Channel6;
        hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_rx.Init.Request             = DMA_REQUEST_12;
        HAL_DMA_Init(&hdma_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_rx);

        HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
#endif
    }
#endif
}

void HAL_UART_MspDeInit(UART_HandleTypeDef * huart)
{
#if ENABLE_UART2
    if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_FORCE_RESET( );
        __HAL_RCC_USART2_RELEASE_RESET( );
        __HAL_RCC_USART2_CLK_DISABLE( );

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
    }
#endif

#if ENABLE_UART4
    if( huart->Instance == USART4 )
    {
        __HAL_RCC_USART4_FORCE_RESET( );
        __HAL_RCC_USART4_RELEASE_RESET( );
        __HAL_RCC_USART4_CLK_DISABLE( );

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

#ifdef LevelShifter_OEpin
        GpioWrite( &LS_OE, 0 );
#endif
    }
#endif
}

#if ENABLE_UART2
bool Uart2Init( void )
{
    HAL_StatusTypeDef status_init;
    HAL_StatusTypeDef status_it;

    Uart2Handle.Instance = USART2;
    Uart2Handle.Init.BaudRate = 115200;
    Uart2Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart2Handle.Init.StopBits = UART_STOPBITS_1;
    Uart2Handle.Init.Parity = UART_PARITY_NONE;
    Uart2Handle.Init.Mode = UART_MODE_TX_RX;
    Uart2Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    Uart2Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    Uart2Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    Uart2Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status_init = HAL_UART_Init(&Uart2Handle);

    status_it = HAL_UART_Receive_IT(&Uart2Handle, &RxData2, 1);

    if ( status_init == HAL_OK && status_it == HAL_OK ) {
        return true;
    }
    else {
        return false;
    }
}

bool Uart2DeInit( void )
{
    HAL_StatusTypeDef status;

    status = HAL_UART_DeInit( &Uart2Handle );

    if ( status == HAL_OK ) {
        return true;
    }
    else {
        return false;
    }
}
#endif

#if ENABLE_UART4
bool Uart4Init( void )
{
    HAL_StatusTypeDef status_init;
    HAL_StatusTypeDef status_it;

    Uart4Handle.Instance = USART4;
    Uart4Handle.Init.BaudRate = 115200;
    Uart4Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart4Handle.Init.StopBits = UART_STOPBITS_1;
    Uart4Handle.Init.Parity = UART_PARITY_NONE;
    Uart4Handle.Init.Mode = UART_MODE_TX_RX;
    Uart4Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    Uart4Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    Uart4Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    Uart4Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status_init = HAL_UART_Init(&Uart4Handle);

#if ENABLE_UART4_INTRRUPT
    status_it = HAL_UART_Receive_IT(&Uart4Handle, DMA_UART4_BUFF, 1);
#else
    status_it = HAL_UART_Receive_DMA(&Uart4Handle, DMA_UART4_BUFF, DMA_BUFFER_SIZE);
#endif
    if ( status_init == HAL_OK && status_it == HAL_OK ) {
        return true;
    }
    else {
        return false;
    }
}

bool Uart4DeInit( void )
{
    HAL_StatusTypeDef status;

    status = HAL_UART_DeInit( &Uart4Handle );

    if ( status == HAL_OK ) {
        return true;
    }
    else {
        return false;
    }
}
#endif

void USART2_IRQHandler( void )
{
#if ENABLE_UART2
    HAL_UART_IRQHandler( &Uart2Handle );
#endif
}
#if ENABLE_UART4_INTRRUPT
void USART4_5_IRQHandler( void )
{
    HAL_UART_IRQHandler( &Uart4Handle );
}
#endif

void DMA1_Channel4_5_6_7_IRQHandler(void)
{
#if ENABLE_UART4
    HAL_DMA_IRQHandler(Uart4Handle.hdmatx);
    HAL_DMA_IRQHandler(Uart4Handle.hdmarx);
#endif
}

void DMA1_Channel2_3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(UartHandle.hdmatx);
    HAL_DMA_IRQHandler(UartHandle.hdmarx);
}
