/*
 * Copyright (c) 2016 - 2017 AcSiP Tech Inc. All rights reserved.
 * LoRa S76S/S78S module
 */

/*!
 * \brief Before entering power saving, it needs to be called by main.o
 *
 * \param [IN] WakeUpTime: The total power-saving time of what we want. Unit: s
 * \param [IN] GPIO Type (e.g. GPIOA, GPIOB, ...)
 */
void PowerSaving( unsigned int WakeUpTime, GPIO_TypeDef *WakeUpGPIO );
