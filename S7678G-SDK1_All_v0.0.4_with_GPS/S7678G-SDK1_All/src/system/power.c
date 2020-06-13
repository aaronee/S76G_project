/*
 * Copyright (c) 2016 - 2017 AcSiP Tech Inc. All rights reserved.
 * LoRa S76S/S78S module
 */

#include "board.h"
#include "utilities.h"
#include "power.h"

#if defined(USE_BAND_470)
#else
extern __IO bool LoRaOscType;    // true for TCXO, false for Crystal, default is false.
#endif

#if ENABLE_POWER_SAVING

/*!
 * \brief By using external pin to wake up RTC power saving mode
 *
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
static void PowerSavingWakeUpPin( GPIO_TypeDef *GPIOx )
{
    GPIO_InitTypeDef GPIO_Struct;

    //Set wake-up pin UART1 & UART2
    GPIO_Struct.Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_Struct.Mode  = GPIO_MODE_IT_RISING;
    GPIO_Struct.Pull  = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init( GPIOx, &GPIO_Struct );
}

/*!
 * \brief Before entering power saving, it needs to be called
 *
 * \param [IN] WakeUpTime: The total power-saving time of what we want. Unit: s
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
static void PowerSavingStopMode( RTC_HandleTypeDef *hrtc, unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO )
{
    GPIO_InitTypeDef GPIO_Struct;

    //Sx1276 sleep first
    SX1276SetSleep( );
    //Sx1276 transceiver set as sleep mode
    SX1276Write( REG_OPMODE, SX1276Read(REG_OPMODE) & 0xF8 );

    //Set power saving duration by RTC
    RtcWakeUpTimeSetting( hrtc, WakeUpTime );

#if defined(LevelShifter_OEpin) && defined(ENABLE_UART4)
    Uart4DeInit();
    DelayMs(5);
#endif

    //relase all pins but PB_2 (GPS_RST_X)
    GPIO_Struct.Pin = GPIO_PIN_All;
    GPIO_Struct.Mode = GPIO_MODE_ANALOG;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_Struct);
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_Init(GPIOE, &GPIO_Struct);
    HAL_GPIO_Init(GPIOH, &GPIO_Struct);

    GPIO_Struct.Pin &= (~GPIO_PIN_2);
    HAL_GPIO_Init(GPIOB, &GPIO_Struct);

    //GPS UART4 TX & RX still not change
    //Level Shifter OE pin still not change, PC6
    GPIO_Struct.Pin = GPIO_PIN_All;
    GPIO_Struct.Pin &= (~GPIO_PIN_10);
    GPIO_Struct.Pin &= (~GPIO_PIN_11);
#ifdef LevelShifter_OEpin
    GPIO_Struct.Pin &= (~GPIO_PIN_6);
#endif
    HAL_GPIO_Init(GPIOC, &GPIO_Struct);

    //GPIO interrupt PIN can be not analog PIN
    if( WakeUpGPIO == GPIOA ) {
        PowerSavingWakeUpPin( WakeUpGPIO );
    }

#if defined( REGION_CN470 ) || defined( REGION_EU433 )
#elif defined( LORA_USE_TCXO )
    if(LoRaOscType == false) {
        // for LoRa sx1276 TCXO OE Pin
        GPIO_Struct.Pin = GPIO_PIN_7;
        GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Struct.Pull = GPIO_NOPULL;
        GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_Struct);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    } else {
        // because PC1 = high for LoRa Crystal, need be careful leakage current
        GPIO_Struct.Pin =  GPIO_PIN_1;
        GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Struct.Pull = GPIO_PULLUP;
        GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &GPIO_Struct);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    }
#endif

    //Sx1276 SPI PIN can be not analog PIN
    GPIO_Struct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Struct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull  = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOB, &GPIO_Struct );
    //Sx1276 SPI power saving setting
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_15, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_12, GPIO_PIN_SET );

    //Enter Power Saving Mode
    HAL_PWREx_EnableUltraLowPower();
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );

    //RTC & GPIO interrupt do nothing, S76 will wake up here.
    BoardInitMcu( );

#if defined( REGION_CN470 ) || defined( REGION_EU433 )
#elif defined( LORA_USE_TCXO )
    SX1276SetOscillator();
#endif
    SX1276SetStby( );
    //Sx1276 back to stand by mode
    SX1276Write( REG_OPMODE, SX1276Read(REG_OPMODE) | 0x01 );
}

void PowerSaving(unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO)
{
    RTC_HandleTypeDef *hrtc;

    __HAL_RCC_TIM3_CLK_DISABLE();

    //RCC Config for Power Saving
    RtcRccPowerSavingConfig( );

    //RTC Config for Power Saving
    hrtc = RtcPowerSavingConfig( );

    //Enter Stop Mode
    PowerSavingStopMode( hrtc, WakeUpTime, WakeUpGPIO );
}
#endif //ENABLE_POWER_SAVING

