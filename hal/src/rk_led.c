/******************************************************************************
 * @file    rk_led.c
 * @brief
 * @details in header file
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_led.h"
#include "main.h"
/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ LOCAL DATA ******************************************************/

/************ LOCAL FUNCTION DECLARATION **************************************/

/************ LOCAL FUNCTION DEFINITION ***************************************/

/************ PUBLIC FUNCTION DEFINITION **************************************/

void RKL_Led(uint8_t status)
{
    switch(status)
    {
    case 0u:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      break;
    case 1u:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
    }
}
