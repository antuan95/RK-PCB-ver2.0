/******************************************************************************
 * @file    app.c
 * @brief   Application source file
 * @details This file contains the implementation of the application functions.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include <stdint.h>
#include "app.h"
#include "rk_timer.h"
#include "rk_can.h"
#include "rk_led.h"
#include "enc.h"
#include "switches.h"
#include "comm_processing.h"
#include "position_sensor.h"

/************ LOCAL DEFINES ***************************************************/
#define APP_BIT_OFFSET_SWITCHES_FROM_RFID         5u
/************ TYPE DEFS *******************************************************/
/**
 * @brief Structure to hold the data from the RFID module.
 */
typedef struct
{
  uint8_t rfid_tag;                         /**< RFID tag identifier */
  uint8_t rfid_version[CP_TAG_VER_SIZE];    /**< RFID version data */
  uint8_t value_sensors;                    /**< Sensor values */
} APP_RFID_TypeDef;

/************ LOCAL DATA ******************************************************/
/**
 * @brief State of the application.
 */
APP_RFID_TypeDef APP_RFID;

/************ LOCAL FUNCTION DECLARATION **************************************/
/**
 * @brief Reset the application data to default values.
 */
static void APP_Reset_Data(void);

/**
 * @brief Callback function for when RFID data is ready.
 *
 * @param data_rfid Pointer to the RFID data structure.
 */
static void APP_RFID_Data_Ready(CP_data_rfid_TypeDef *data_rfid);

/**
 * @brief Callback function for when main data is ready.
 *
 * @param data_main Pointer to the main data structure.
 */
static void APP_MAIN_Data_Ready(CP_data_main_TypeDef *data_main);

/**
 * @brief Get sensor values.
 *
 * @return Sensor value byte.
 */
static uint8_t APP_Get_Sensors(void);

/**
 * @brief Send data to the main domain.
 */
static void APP_Send_To_Main(void);

/************ LOCAL FUNCTION DEFINITION ***************************************/
/**
 * @brief Reset the application data to default values.
 */
static void APP_Reset_Data(void)
{
  APP_RFID.rfid_tag = 0u;
  for (uint8_t i = 0; i < CP_TAG_VER_SIZE; i++)
  {
    APP_RFID.rfid_version[i] = 0u;
  }
  APP_RFID.value_sensors = 0u;
}

/**
 * @brief Callback function for when RFID data is ready.
 *
 * @param data_rfid Pointer to the RFID data structure.
 */
static void APP_RFID_Data_Ready(CP_data_rfid_TypeDef *data_rfid)
{
  APP_RFID.rfid_tag = data_rfid->tag_uid;
  for (uint8_t i = 0; i < CP_TAG_VER_SIZE; i++)
  {
    APP_RFID.rfid_version[i] = data_rfid->tag_version[i];
  }
  APP_RFID.value_sensors = data_rfid->value_sensors;
}

/**
 * @brief Callback function for when main data is ready.
 *
 * @param data_main Pointer to the main data structure.
 */
static void APP_MAIN_Data_Ready(CP_data_main_TypeDef *data_main)
{
  switch (data_main->cmd)
  {
    case CP_GET_STATE:
      APP_Send_To_Main();
      break;
    case CP_LED:
      RKL_Led(data_main->value);
      break;
    default:
      break;
  }
}

/**
 * @brief Get switches values.
 *
 * @return Switches value byte.
 */
static uint8_t APP_Get_Sensors(void)
{
  uint8_t switches = SW_Get_Switches_Byte() | ((APP_RFID.value_sensors) << APP_BIT_OFFSET_SWITCHES_FROM_RFID);
  return switches;
}

/**
 * @brief Send data to the main domain.
 */
static void APP_Send_To_Main(void)
{
  CP_response_main_TypeDef response = {0};
  response.tag = APP_RFID.rfid_tag;
  response.enc_value = ENC_Get_Value();
  response.enc_button_counter = SW_Get_Button_Counter();
  response.sensors = APP_Get_Sensors();
  response.motor_position = PS_Get_Position(MT_MOTOR);
  response.arm_position = PS_Get_Position(MT_ARM);
  CP_Send_Data_To_Main(response);
}

/************ PUBLIC FUNCTION DEFINITION **************************************/
void APP_Init(void)
{
  APP_Reset_Data();
  CP_Register_RFID_Data_Ready_Callback(APP_RFID_Data_Ready);
  CP_Register_Main_Data_Ready_Callback(APP_MAIN_Data_Ready);
  SW_Init();
  ENC_Init();
  CP_Init();
  PS_Init();
}

void APP_Run(void)
{

}
