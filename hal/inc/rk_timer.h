/******************************************************************************
 * @file    rk_timer.h
 * @brief   Header file for the timer management module
 * @details This file contains the declarations of functions and data types for
 *          handling system tick and debounce timers with callback support.
 ******************************************************************************/
#ifndef RK_TIMER_H_
#define RK_TIMER_H_

/************ INCLUDES ********************************************************/

#ifdef STM32G031xx
#include "stm32g0xx_hal.h"
#endif

#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif

#ifdef STM32L431xx
#include "stm32l4xx_hal.h"
#endif

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/**
 * @brief Enumeration for timer callback types
 */
typedef enum
{
  TM_NO_TYPE,    ///< No callback type
  TM_TICK_1MS,   ///< 1 millisecond tick callback
  TM_DEBOUNCE    ///< Debounce callback
} TM_timer_type_TypeDef;

/**
 * @brief Type definition for callback functions
 */
typedef void(*TM_Callback)(void);

/**
 * @brief Timer instance structure
 */
typedef struct
{
  uint16_t period;                      ///< Timer period
  uint16_t time_left_until_callback;    ///< Time left until callback
  TM_timer_type_TypeDef type;           ///< Timer callback type
  TM_Callback callback;                 ///< Timer callback function
} TM_timer_instance_TypeDef;

/************ INTERFACE *******************************************************/

/**
 * @brief Initialize the timer module.
 */
void TM_Init(void);

/**
 * @brief Initialize a free-running timer instance.
 *
 * @param instance Pointer to the timer instance to initialize.
 */
void TM_Init_Instance_Free_Running(TM_timer_instance_TypeDef *instance);

/**
 * @brief Register a debounce callback function.
 *
 * @param callback The debounce callback function to register.
 */
void TM_Register_Callback_Debounce(TM_Callback callback);

/**
 * @brief Start the debounce timer.
 */
void TM_Start_Timer_Debounce(void);

#endif /* RK_TIMER_H_ */
