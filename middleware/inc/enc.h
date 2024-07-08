/******************************************************************************
 * @file    enc.h
 * @brief   Header file for encoder handling and debouncing
 * @details Provides function prototypes and includes necessary for encoder
 *          initialization and handling with debounce functionality.
 ******************************************************************************/

#ifndef __ENC_H
#define __ENC_H

/************ INCLUDES ********************************************************/
#include <stdint.h>
#include <stdbool.h>
#ifdef STM32G031xx
#include "stm32g0xx_hal.h"
#endif
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif
#ifdef STM32L431xx
#include "stm32l4xx_hal.h"
#endif

/************ FUNCTION PROTOTYPES *********************************************/
/**
 * @brief Initializes the encoder.
 */
void ENC_Init(void);

/**
 * @brief Handler for encoder pin interrupts.
 */
void ENC_Handler(void);

/**
 * @brief Returns the value of the incremental encoder.
 * @return Encoder value.
 */
uint8_t ENC_Get_Value(void);

/**
 * @brief Debounce handling for encoder pin interrupts.
 * @param tim Timer handle for debounce.
 * @param GPIO_Pin The pin that triggered the interrupt.
 */
void ENC_Debounce(uint16_t GPIO_Pin);

#endif /* __ENC_H */
