/******************************************************************************
 * @file    rk_led.h
 * @brief   Header file for LED control module.
 * @details Provides function declarations for controlling LEDs.
 ******************************************************************************/
#ifndef RK_LED_H_
#define RK_LED_H_

/************ INCLUDES ********************************************************/

#ifdef STM32G031xx
    #include "stm32g031xx.h"
#endif
#ifdef STM32F407xx
    #include "stm32f4xx_hal.h"
#endif
#ifdef STM32L431xx
    #include "stm32l4xx_hal.h"
#endif

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ INTERFACE *******************************************************/

/**
 * @brief Control the LED status.
 *
 * @param status The status to set for the LED.
 */
void RKL_Led(uint8_t status);

#endif /* RK_LED_H_ */
