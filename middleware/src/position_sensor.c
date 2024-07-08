/******************************************************************************
 * @file    position_sensor.c
 * @brief   Position sensor module implementation
 * @details This file contains the implementation of functions to interface
 *          with position sensors and calculate position based on raw data.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_timer.h"
#include "position_sensor.h"

/************ LOCAL DEFINES ***************************************************/
#define PS_WINDOW_SIZE              10u  ///< Size of the moving average window
#define PS_SENSOR_POLL_PERIOD       100u ///< Polling period

/************ TYPE DEFS *******************************************************/
typedef struct
{
    uint16_t raw_position[PS_WINDOW_SIZE];
} PS_raw_position_TypeDef;

/************ LOCAL DATA ******************************************************/
PS_raw_position_TypeDef PS_raw_position[MT_ID_COUNT] = {0u};  ///< Raw position data for each sensor

/************ LOCAL FUNCTION DECLARATION **************************************/
static void PS_Read_Raw_Position(void);
static void PS_Add_To_Moving_Average_Filter(uint16_t *buffer, uint16_t new_value);

/************ LOCAL FUNCTION DEFINITION ***************************************/
/**
 * @brief Add a new value to the moving average filter buffer
 * @param buffer Pointer to the buffer
 * @param new_value New value to add to the buffer
 */
static void PS_Add_To_Moving_Average_Filter(uint16_t *buffer, uint16_t new_value)
{
    // Shift all elements in the buffer to the left
    for (uint8_t i = 1u; i < PS_WINDOW_SIZE; i++)
    {
        buffer[i - 1u] = buffer[i];
    }
    // Add the new value at the end of the buffer
    buffer[PS_WINDOW_SIZE - 1u] = new_value;
}

/************ PUBLIC FUNCTION DEFINITION **************************************/
/**
 * @brief Initialize the position sensor module
 * @details Initializes timer instance to periodically read raw position data.
 */
void PS_Init(void)
{
    TM_timer_instance_TypeDef timer_instance =
    {
        .period = PS_SENSOR_POLL_PERIOD,
        .type = TM_TICK_1MS,
        .callback = PS_Read_Raw_Position
    };
    TM_Init_Instance_Free_Running(&timer_instance);
}

/**
 * @brief Read raw position data from all sensors
 * @details Reads raw position data from each sensor using MM_Read_Register.
 *          Filters the raw data using a moving average filter.
 */
void PS_Read_Raw_Position(void)
{
    for (MT_identifier_TypeDef i = MT_MOTOR; i < MT_ID_COUNT; i++)
    {
        uint16_t position = 0;
        uint8_t pos_hi = 0;
        uint8_t pos_low = 0;

        // Read high and low bytes of position data
        MT_Read_Register(i, 0x03u, &pos_hi);
        MT_Read_Register(i, 0x04u, &pos_low);

        // Combine high and low bytes to form position
        position = ((((uint16_t)pos_hi << 8u) | (uint16_t)(pos_low)) >> 2u);

        // Add new position to moving average filter
        PS_Add_To_Moving_Average_Filter(PS_raw_position[i].raw_position, position);
    }
}

/**
 * @brief Calculate the averaged position from raw data
 * @param Id Identifier of the sensor
 * @return Calculated position based on averaged raw data
 */
uint8_t PS_Get_Position(MT_identifier_TypeDef Id)
{
    uint8_t position = 0;

    // Calculate the moving average
    uint32_t sum = 0;
    for (uint8_t i = 0; i < PS_WINDOW_SIZE; i++)
    {
        sum += PS_raw_position[Id].raw_position[i];
    }
    uint16_t average = sum / PS_WINDOW_SIZE;

    // Determine position based on average value
    if ((average > 0u) && (average <= 4000u))
    {
        position = 1u;
    }
    else if ((average > 4000u) && (average <= 10000u))
    {
        position = 2u;
    }
    else if ((average > 10000u) && (average < 16384u))
    {
        position = 3u;
    }

    return position;
}
