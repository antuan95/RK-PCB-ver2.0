/******************************************************************************
 * @file    rk_timer.c
 * @brief   Timer management module
 * @details This file contains the implementation of timer management functions
 *          for handling system tick and debounce timers with callback support.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_timer.h"

/************ LOCAL DEFINES ***************************************************/
#define TM_MAX_NUMBER_INSTANCES    10u  ///< Maximum number of timer instances

/************ TYPE DEFS *******************************************************/

/************ LOCAL DATA ******************************************************/
extern TIM_HandleTypeDef htim6;  ///< Handle for system tick timer
extern TIM_HandleTypeDef htim7;  ///< Handle for debounce timer

TIM_HandleTypeDef *TM_systick_tim = &htim6;  ///< Pointer to the system tick timer handle
TIM_HandleTypeDef *TM_debounce_tim = &htim7; ///< Pointer to the debounce timer handle

TM_timer_instance_TypeDef TM_instance[TM_MAX_NUMBER_INSTANCES]; ///< Array of timer instances
uint8_t TM_instance_index = 0u; ///< Index to track the number of registered instances

TM_Callback TM_Callback_Debounce = NULL; ///< Callback for debounce timer

/************ LOCAL FUNCTION DECLARATION **************************************/

/************ LOCAL FUNCTION DEFINITION ***************************************/

/************ PUBLIC FUNCTION DEFINITION **************************************/

void TM_Init(void)
{
  for(uint8_t i = 0u; i < TM_MAX_NUMBER_INSTANCES; i++)
  {
    TM_instance[i].callback = NULL;
    TM_instance[i].type = TM_NO_TYPE;
    TM_instance[i].period = 0u;
  }
  HAL_TIM_Base_Start_IT(TM_systick_tim);
}

void TM_Init_Instance_Free_Running(TM_timer_instance_TypeDef *instance)
{
  if(TM_instance_index < TM_MAX_NUMBER_INSTANCES)
  {
    TM_instance[TM_instance_index].period = instance->period;
    TM_instance[TM_instance_index].time_left_until_callback = instance->period;
    if(instance->callback != NULL)
    {
      TM_instance[TM_instance_index].callback = instance->callback;
      TM_instance[TM_instance_index].type = instance->type;
    }
    TM_instance_index++;
  }
}

void TM_Register_Callback_Debounce(TM_Callback callback)
{
  if(callback != NULL)
  {
    TM_Callback_Debounce = callback;
  }
}


void TM_Start_Timer_Debounce(void)
{
  __HAL_TIM_CLEAR_IT(TM_debounce_tim, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(TM_debounce_tim, 0);
  HAL_TIM_Base_Start_IT(TM_debounce_tim);             // Start debounce timer
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == TM_systick_tim)
  {
    for(uint8_t i = 0u; i < TM_instance_index; i++)
    {
      if(TM_instance[i].type == TM_TICK_1MS)
      {
        TM_instance[i].time_left_until_callback--;
        if((TM_instance[i].time_left_until_callback == 0u) && (TM_instance[i].callback != NULL))
        {
          TM_instance[i].callback();
          TM_instance[i].time_left_until_callback = TM_instance[i].period;
        }
      }
    }
  }
  else if(htim == TM_debounce_tim)
  {
    if(TM_Callback_Debounce != NULL)
    {
      TM_Callback_Debounce();
    }
  }
}
