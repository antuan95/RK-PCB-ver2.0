/******************************************************************************
 * @file    enc.c
 * @brief   Encoder handling and debouncing implementation
 * @details Provides functions for initializing and handling encoder inputs with
 *          debouncing.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "enc.h"
#include "rk_timer.h"
#include "main.h"

/************ LOCAL DEFINES ***************************************************/
#define ENC_DEBOUNCE_READY      0u
#define ENC_DEBOUNCE_BUSY       1u

/************ TYPE DEFS *******************************************************/
typedef struct
{
    int8_t state;
    bool pinA_Value;
    bool pinB_Value;
    uint8_t counter;
} ENC_Instance_TypeDef;

/************ EXTERNAL VARIABLES **********************************************/
extern TIM_HandleTypeDef htim7;

/************ LOCAL DATA ******************************************************/
TIM_HandleTypeDef *ENC_tim_debounce = &htim7;
volatile ENC_Instance_TypeDef ENC_Instance;           // Encoder instance
volatile bool ENC_debounce;                           // Debounce status
uint16_t ENC_pin;                                     // Triggered pin

/************ LOCAL FUNCTION DECLARATION **************************************/
static void ENC_Set_Count(int8_t state);   // Set counter value

/************ LOCAL FUNCTION DEFINITION ***************************************/
static void ENC_Set_Count(int8_t state)   // Set counter value
{
    if (state == 4u || state == -4u)  // If the state has reached the specified increment
    {
        ENC_Instance.counter += (int8_t)(state / 4u);  // Increment/decrement the counter
    }
}

/************ PUBLIC FUNCTION DEFINITION **************************************/
void ENC_Init(void)
{
  TM_Register_Callback_Debounce(ENC_Handler);
}

uint8_t ENC_Get_Value(void) // Get encoder value
{
    return ENC_Instance.counter;
}

void ENC_Debounce(uint16_t GPIO_Pin)
{
    if (ENC_debounce == ENC_DEBOUNCE_READY)
    {
        ENC_pin = GPIO_Pin;                     // Save the triggered pin value
        TM_Start_Timer_Debounce();
        ENC_debounce = ENC_DEBOUNCE_BUSY;       // Block signal processing during debounce
    }
}

void ENC_Handler()
{
    HAL_TIM_Base_Stop_IT(ENC_tim_debounce);

    ENC_Instance.pinA_Value = HAL_GPIO_ReadPin(EncA_GPIO_Port, EncA_Pin);   // Get the state of pins A and B
    ENC_Instance.pinB_Value = HAL_GPIO_ReadPin(EncB_GPIO_Port, EncB_Pin);

    if (ENC_pin == EncA_Pin)  // If interrupt came from pin A
    {
        if (((ENC_Instance.state == 0u) && (ENC_Instance.pinA_Value == 0u) && (ENC_Instance.pinB_Value != 0u)) ||
            ((ENC_Instance.state == 2u) && (ENC_Instance.pinA_Value != 0u) && (ENC_Instance.pinB_Value == 0u)))
        {
            ENC_Instance.state++;
        }
        else if (((ENC_Instance.state == -1u) && (ENC_Instance.pinA_Value == 0u) && (ENC_Instance.pinB_Value == 0u)) ||
                 ((ENC_Instance.state == -3u) && (ENC_Instance.pinA_Value != 0u) && (ENC_Instance.pinB_Value != 0u)))
        {
            ENC_Instance.state--;
        }
    }
    if (ENC_pin == EncB_Pin)  // If interrupt came from pin B
    {
        if (((ENC_Instance.state == 1u) && (ENC_Instance.pinA_Value == 0u) && (ENC_Instance.pinB_Value == 0u)) ||
            ((ENC_Instance.state == 3u) && (ENC_Instance.pinA_Value != 0u) && (ENC_Instance.pinB_Value != 0u)))
        {
            ENC_Instance.state++;
        }
        else if (((ENC_Instance.state == 0u) && (ENC_Instance.pinA_Value != 0u) && (ENC_Instance.pinB_Value == 0u)) ||
                 ((ENC_Instance.state == -2u) && (ENC_Instance.pinA_Value == 0u) && (ENC_Instance.pinB_Value != 0u)))
        {
            ENC_Instance.state--;
        }
    }
    ENC_Set_Count(ENC_Instance.state); // Check if a full step of 4 signal changes (2 pulses) occurred

    if ((ENC_Instance.pinA_Value != 0u) && (ENC_Instance.pinB_Value != 0u) && (ENC_Instance.state != 0u))
        ENC_Instance.state = 0u; // If something went wrong, reset the state to the initial state

    ENC_debounce = ENC_DEBOUNCE_READY;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)    // Handle any edge-triggered interrupt
{
    if ((pin == EncA_Pin) || (pin == EncB_Pin))
    {
        ENC_Debounce(pin);
    }
}
