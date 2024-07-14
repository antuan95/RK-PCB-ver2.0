/******************************************************************************
 * @file    comm_processing.h
 * @brief   Header file for communication processing
 *
 * @details This file contains definitions, macros, and function prototypes for
 *          handling and processing communication data between the main and RFID modules.
 ******************************************************************************/

#ifndef _COMM_PROCESSING_H
#define _COMM_PROCESSING_H

/************ INCLUDES ********************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "switches.h"
#include "enc.h"
#include "rk_uart.h"
#include "rk_mt6701.h"

/************ LOCAL DEFINES ***************************************************/

// Main communication definitions
#define CP_PREAMBLE                     0xAAu      ///< Preamble byte for main communication
#define CP_SIZE_OFFSET                  4u         ///< Size offset for main communication
#define CP_TX_PAYLOAD_SIZE              6u         ///< Transmission payload size for main communication
#define CP_TX_SIZE                      (CP_TX_PAYLOAD_SIZE + CP_SIZE_OFFSET) ///< Total transmission size
#define CP_GET_STATE                    0x71u      ///< Command to get state
#define CP_LED                          0x72u      ///< Command to control LED
#define CP_ADDRESS                      1u         ///< Address for main communication

// RFID communication definitions
#define CP_RF_PREAMBLE                  0x43u      ///< Preamble byte for RFID communication
#define CP_CMD_VERSION                  0x01u      ///< Command for RFID version
#define CP_CMD_TAG_UID                  0x02u      ///< Command for RFID tag UID
#define CP_RF_SIZE_OFFSET               3u         ///< Size offset for RFID communication (preamble, packet length, CRC)
#define CP_RF_TX_PAYLOAD_SIZE_VERSION   1u         ///< Payload size for RFID version request
#define CP_RF_TX_PAYLOAD_SIZE_TAG       1u         ///< Payload size for RFID tag request
#define CP_RFID_PAYLOAD_VERSION_OFFSET  3u         ///< Data offset for RFID version payload
#define CP_RFID_PAYLOAD_TAG_OFFSET      4u         ///< Data offset for RFID tag payload
#define CP_TAG_VER_SIZE                 3u         ///< Size of the RFID tag version array

/************ TYPE DEFS *******************************************************/

/**
 * @brief Error types for main communication
 */
typedef enum
{
  CP_DATA_NO_ERROR = 0u,   ///< No error
  CP_ADDRESS_ERROR = 1u,   ///< Address error
  CP_SIZE_ERROR = 2u,      ///< Size error
  CP_CRC_ERROR = 3u,       ///< CRC error
  CP_PREAMBLE_ERROR = 4u   ///< Preamble error
} CP_error_TypeDef;

/**
 * @brief Error types for RFID communication
 */
typedef enum
{
  CP_RF_DATA_NO_ERROR = 0u,    ///< No error
  CP_RF_COMMAND_ERROR = 1u,    ///< Command error
  CP_RF_SIZE_ERROR = 2u,       ///< Size error
  CP_RF_CRC_ERROR = 3u,        ///< CRC error
  CP_RF_PREAMBLE_ERROR = 4u    ///< Preamble error
} CP_error_RF_TypeDef;

/**
 * @brief Data structure for main communication
 */
typedef struct
{
  uint8_t addr;    ///< Address
  uint8_t cmd;     ///< Command
  uint8_t value;   ///< Value
} CP_data_main_TypeDef;

/**
 * @brief Data structure for main communication response
 */
typedef struct
{
  uint8_t tag;                    ///< Tag information
  uint8_t sensors;                ///< Sensor information
  uint8_t enc_button_counter;     ///< Encoder button counter
  uint8_t enc_value;              ///< Encoder value
  uint8_t arm_position;           ///< Arm position
  uint8_t motor_position;         ///< Motor position
} CP_response_main_TypeDef;

/**
 * @brief Data structure for RFID communication
 */
typedef struct
{
  uint8_t cmd;                        ///< Command
  uint8_t uid_len;                    ///< Length of UID
  uint8_t tag_uid;                    ///< UID of the tag
  uint8_t tag_version[CP_TAG_VER_SIZE]; ///< Version of the tag
  uint8_t value_sensors;              ///< Sensor values
} CP_data_rfid_TypeDef;

/**
 * @brief Callback function type for RFID data readiness
 */
typedef void(*CP_Callback_RFID_Data_Ready)(CP_data_rfid_TypeDef *data_rfid);

/**
 * @brief Callback function type for main data readiness
 */
typedef void(*CP_Callback_Main_Data_Ready)(CP_data_main_TypeDef *data_main);

/************ INTERFACE *******************************************************/

/**
 * @brief Initialize communication processing
 */
void CP_Init(void);

/**
 * @brief Send a request for the RFID version
 * @param domain Pointer to the RFID domain
 */
void CP_Send_Request_RF_Version(RKU_domain_TypeDef *domain);

/**
 * @brief Send a request for the RFID tag
 * @param domain Pointer to the RFID domain
 */
void CP_Send_Request_RF_Tag(RKU_domain_TypeDef *domain);

/**
 * @brief Send the current state over UART
 * @param domain Pointer to the domain
 */
void CP_Send_State(RKU_domain_TypeDef *domain);

/**
 * @brief Send the current RFID state over UART
 * @param domain Pointer to the RFID domain
 */
void CP_Send_State_RF(RKU_domain_TypeDef *domain);

/**
 * @brief Initialize a communication domain
 * @param huart Pointer to the UART handle
 * @param name Name of the domain
 */
void CP_Init_Domain(UART_HandleTypeDef *huart, RKU_domainID_TypeDef name);

/**
 * @brief Register a callback function for RFID data readiness
 * @param callback Pointer to the callback function
 */
void CP_Register_RFID_Data_Ready_Callback(CP_Callback_RFID_Data_Ready callback);

/**
 * @brief Register a callback function for main data readiness
 * @param callback Pointer to the callback function
 */
void CP_Register_Main_Data_Ready_Callback(CP_Callback_Main_Data_Ready callback);

/**
 * @brief Send data to the main domain
 * @param response Data to send
 */
void CP_Send_Data_To_Main(CP_response_main_TypeDef response);

#endif /* _COMM_PROCESSING_H */
