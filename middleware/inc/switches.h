/******************************************************************************
 * @file    switches.h
 * @brief   Switch handling and debouncing
 * @details This file contains definitions and function prototypes for
 *          initializing, reading, and debouncing switches and buttons.
 ******************************************************************************/

#ifndef __SWITCHES_H
#define __SWITCHES_H

/************ INCLUDES ********************************************************/
#include <stdint.h>
#include <stdbool.h>

/************ DEFINES *********************************************************/

/************ TYPEDEFS ********************************************************/
/**
 * @brief Possible states of a switch
 */
typedef enum
{
    SW_PRESSED      = 0,  ///< Switch is pressed
    SW_RELEASED     = 1   ///< Switch is released
} SW_switch_state_TypeDef;

/**
 * @brief Names of the switches
 */
typedef enum
{
    SW_BUTTON = 0,        ///< Button switch
    SW_ARM_SW,            ///< Arm switch
    SW_MOTOR_SW,          ///< Motor switch
    SW_HALL_SENSOR,       ///< Hall sensor switch
    SW_SWITCHES_NUMBER    ///< Number of switches
} SW_switch_name_TypeDef;

/************ FUNCTION PROTOTYPES *********************************************/
/**
 * @brief Initialize switch structures
 *
 * This function initializes the ports and pins in the switch structures and
 * registers the SysTick callback for switch debouncing.
 */
void SW_Init(void);

/**
 * @brief Get the current state of a switch
 *
 * @param name The name of the switch
 * @return The state of the switch (SW_PRESSED or SW_RELEASED)
 */
SW_switch_state_TypeDef SW_Get_Switch_State(SW_switch_name_TypeDef name);

/**
 * @brief Get the press counter value of the button
 *
 * @return The button press counter
 */
uint8_t SW_Get_Button_Counter(void);

/**
 * @brief SysTick callback for switch debouncing
 *
 * This function manages the period for polling switches and triggers the
 * switch check.
 */
void SW_Systick_Callback_Switches(void);

/**
 * @brief Get a byte representing the states of all switches and the LED
 *
 * @return A byte with switch states and LED status
 */
uint8_t SW_Get_Switches_Byte(void);

#endif /* __SWITCHES_H */
