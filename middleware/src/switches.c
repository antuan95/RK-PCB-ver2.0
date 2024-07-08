/******************************************************************************
 * @file    switches.c
 * @brief   Switch handling and debouncing
 * @details This file contains functions to initialize, read, and debounce
 *          switches and buttons.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "switches.h"
#include "rk_timer.h"
#include "main.h"

/************ DEFINES *********************************************************/
#define SW_CHECK_SWITCH_PERIOD      10u          ///< Polling period for switches in SysTick intervals (10ms)
#define SW_NUMBER_OF_READS          3u           ///< Number of consecutive reads required for debouncing
/************ TYPE DEFS *******************************************************/
/**
 * @brief Structure for a switch
 */
typedef struct
{
    uint16_t pin;                   ///< Pin number
    GPIO_TypeDef* port;             ///< Port
    SW_switch_state_TypeDef state;  ///< Current state of the switch
    uint8_t checkCounter;           ///< Temporary counter for debouncing
    uint8_t previousState;          ///< Previous state read from the pin
    uint8_t counter;                ///< Press counter (for button)
} SW_switch_TypeDef;

/************ LOCAL DATA ******************************************************/
volatile SW_switch_TypeDef SW_switches[SW_SWITCHES_NUMBER] = {0};  ///< Array of switch structures
volatile uint8_t SW_checkSwitchPeriod = SW_CHECK_SWITCH_PERIOD;    ///< Period for polling switches (10ms per SysTick period)

/************ LOCAL FUNCTION DECLARATION **************************************/
static void SW_Check_Switches(void);  ///< Check the states of all switches

/************ PUBLIC FUNCTION DEFINITION **************************************/

/**
 * @brief Initialize switches
 *
 * Fills in the ports and pins in the switch structures and registers the
 * SysTick callback for switch debouncing.
 */
void SW_Init(void)
{
    SW_switches[SW_BUTTON].pin = Button_Pin;
    SW_switches[SW_BUTTON].port = Button_GPIO_Port;
    SW_switches[SW_ARM_SW].pin = Arm_sw_Pin;
    SW_switches[SW_ARM_SW].port = Arm_sw_GPIO_Port;
    SW_switches[SW_MOTOR_SW].pin = Motor_sw_Pin;
    SW_switches[SW_MOTOR_SW].port = Motor_sw_GPIO_Port;
    SW_switches[SW_HALL_SENSOR].pin = Hall_Sensor_Pin;
    SW_switches[SW_HALL_SENSOR].port = Hall_Sensor_GPIO_Port;

    TM_timer_instance_TypeDef timer_instance =
    {
        .period = SW_CHECK_SWITCH_PERIOD,
        .type = TM_TICK_1MS,
        .callback = SW_Check_Switches
    };
    TM_Init_Instance_Free_Running(&timer_instance);
}

/************ LOCAL FUNCTION DEFINITION ***************************************/

/**
 * @brief Check the states of all switches
 *
 * Debounces the switch inputs and updates their states and counters.
 */
static void SW_Check_Switches(void)
{
    for(uint8_t i = 0u; i < SW_SWITCHES_NUMBER; i++)  // Check all switches in a loop
    {
        SW_switch_state_TypeDef tempState;
        tempState = HAL_GPIO_ReadPin(SW_switches[i].port, SW_switches[i].pin); // Read pin state
        if(tempState == SW_switches[i].previousState)  // If state is the same as the previous read
        {
            SW_switches[i].checkCounter++;  // Increment temporary counter
            if(SW_switches[i].checkCounter == SW_NUMBER_OF_READS)  // If state is consistent across multiple reads
            {
                if((SW_switches[i].state == SW_RELEASED) && (tempState == SW_PRESSED))
                {
                    SW_switches[i].counter++;  // Increment press counter for button
                }
                SW_switches[i].state = SW_switches[i].previousState;  // Update switch state
                SW_switches[i].checkCounter = 0u;  // Reset temporary counter
            }
        }
        else  // If state differs from the previous read
        {
            SW_switches[i].previousState = tempState;  // Update previous state
            SW_switches[i].checkCounter = 0u;  // Reset temporary counter
        }
    }
}

/**
 * @brief Get the current state of a switch
 * @param name The name of the switch
 * @return The state of the switch
 */
SW_switch_state_TypeDef SW_Get_Switch_State(SW_switch_name_TypeDef name)
{
    return SW_switches[name].state;
}

/**
 * @brief Get the press counter value of the button
 * @return The button press counter
 */
uint8_t SW_Get_Button_Counter(void)
{
    return SW_switches[SW_BUTTON].counter;
}

/**
 * @brief Get a byte representing the states of all switches and the LED
 * @return A byte with switch states and LED status
 */
uint8_t SW_Get_Switches_Byte(void)
{
    uint8_t byte = 0u;
    GPIO_PinState led_status;
    for(uint8_t i = 0u; i < SW_SWITCHES_NUMBER; i++)
    {
        byte |= (SW_switches[i].state << i);
    }
    led_status = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin);
    byte |= (led_status << SW_SWITCHES_NUMBER);
    return byte;
}
