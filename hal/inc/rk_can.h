/******************************************************************************
 * @file    rk_can.h
 * @brief   Header file for CAN communication module.
 * @details This file contains function prototypes and data type definitions
 *          for CAN communication.
 ******************************************************************************/
#ifndef RK_CAN_H_
#define RK_CAN_H_

/************ INCLUDES ********************************************************/
#ifdef STM32G031xx
    #include "stm32g031xx.h"
#endif
#ifdef STM32F407xx
    #include "stm32f4xx_hal.h"
#endif
#ifdef STM32L431xx
    #include "stm32l4xx_hal.h"
#endif

/************ LOCAL DEFINES ***************************************************/
#define RKC_DATA_LENGTH             8u   /**< Length of CAN data */
#define RKC_ID                      0x321u /**< CAN message ID */

/************ TYPE DEFS *******************************************************/
/**
 * @brief Enumeration for CAN instances.
 */
typedef enum
{
  RKC_MAIN = 0u, /**< Main CAN instance */
  RKC_ID_COUNT   /**< Total number of CAN instances */
} RKC_Identifier_te;

/**
 * @brief Enumeration for CAN status.
 */
typedef enum
{
  RKC_OK,       /**< Operation successful */
  RKC_ERROR     /**< Operation failed */
} RKC_Status_te;

/**
 * @brief Callback function type for CAN RX.
 *
 * @param data Pointer to the received data.
 * @param size Size of the received data.
 */
typedef void(*RKC_RX_Callback)(uint8_t *data, uint8_t size);

/************ INTERFACE *******************************************************/
/**
 * @brief Initialize the CAN communication module.
 */
void RKC_Can_Init(void);

/**
 * @brief Start the specified CAN instance.
 *
 * @param canID Identifier of the CAN instance.
 * @return Status of the operation.
 */
RKC_Status_te RKC_Can_Start(RKC_Identifier_te canID);

/**
 * @brief Set the CAN handle for a specific CAN instance.
 *
 * @param canID Identifier of the CAN instance.
 * @param hcan Pointer to the CAN handle.
 * @return Status of the operation.
 */
RKC_Status_te RKC_Set_CAN_Instance(const RKC_Identifier_te canID, CAN_HandleTypeDef * const hcan);

/**
 * @brief Add a message to the CAN transmit queue.
 *
 * @param canID Identifier of the CAN instance.
 * @param txData Pointer to the data to be transmitted.
 * @return Status of the operation.
 */
RKC_Status_te RKC_Add_Tx_Message(const RKC_Identifier_te canID, uint8_t *txData);

/**
 * @brief Register a callback function for CAN RX.
 *
 * @param callback Function pointer to the callback.
 */
void RKC_Register_RX_Callback(RKC_RX_Callback callback);

#endif /* RK_CAN_H_ */
