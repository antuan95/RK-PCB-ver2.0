/******************************************************************************
 * @file    position_sensor.h
 * @brief   Header file for position sensor module
 *
 * @details in header file
 ******************************************************************************/

#ifndef POSITION_SENSOR_H_
#define POSITION_SENSOR_H_

/************ INCLUDES ********************************************************/
#include "rk_mt6701.h"

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ INTERFACE *******************************************************/

/**
 * @brief Initialize the position sensor module
 */
void PS_Init(void);

/**
 * @brief Get the position from a specific sensor
 * @param Id Identifier of the sensor
 * @return Position value read from the sensor
 */
uint8_t PS_Get_Position(MT_identifier_TypeDef Id);

#endif /* POSITION_SENSOR_H_ */
