/******************************************************************************
 * @file    rk_can.c
 * @brief   Source file for CAN communication module.
 * @details See header file for details.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_can.h"

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/
/**
 * @brief Structure to hold CAN instance information.
 */
typedef struct
{
  CAN_HandleTypeDef *canInstance; /**< Pointer to CAN handle instance */
} RKC_Instance_t;

/************ LOCAL DATA ******************************************************/
/** CAN transmit header configuration */
CAN_TxHeaderTypeDef txHeader =
{
    .DLC = RKC_DATA_LENGTH,           /**< Data length */
    .StdId = RKC_ID,                  /**< Standard identifier */
    .ExtId = 0u,                      /**< Not used */
    .IDE = CAN_ID_STD,                /**< Standard ID */
    .RTR = CAN_RTR_DATA,              /**< Data frame */
    .TransmitGlobalTime = DISABLE     /**< No global time */
};

/** CAN transmit mailbox */
uint32_t RKC_tx_mailbox;

/** Array of CAN instance structures */
RKC_Instance_t sRKC_Instance[RKC_ID_COUNT];

/** Callback function for CAN RX */
RKC_RX_Callback RKC_callback = NULL;

/************ LOCAL FUNCTION DECLARATION **************************************/

/************ LOCAL FUNCTION DEFINITION ***************************************/

/************ PUBLIC FUNCTION DEFINITION **************************************/

RKC_Status_te RKC_Can_Start(RKC_Identifier_te canID)
{
  RKC_Status_te status = RKC_ERROR;
  if((canID < RKC_ID_COUNT) && (sRKC_Instance[canID].canInstance != NULL))
  {
    HAL_CAN_Start(sRKC_Instance[canID].canInstance);
    HAL_CAN_ActivateNotification(sRKC_Instance[canID].canInstance, CAN_IT_RX_FIFO0_MSG_PENDING);

    status = RKC_OK;
  }
  return status;
}

RKC_Status_te RKC_Set_CAN_Instance(const RKC_Identifier_te canID, CAN_HandleTypeDef * const hcan)
{
  RKC_Status_te status = RKC_ERROR;
  if((canID < RKC_ID_COUNT) && (hcan != NULL))
  {
    for(uint8_t i = 0u; i < RKC_ID_COUNT; i++)
    {
      if(i == canID)
      {
        sRKC_Instance[i].canInstance = hcan;
        RKC_Can_Start(canID);
        status = RKC_OK;
      }
    }
  }
  return status;
}

RKC_Status_te RKC_Add_Tx_Message(const RKC_Identifier_te canID, uint8_t *txData)
{
  RKC_Status_te status = RKC_ERROR;
  if((canID < RKC_ID_COUNT) && (sRKC_Instance[canID].canInstance != NULL))
  {
    if(HAL_CAN_AddTxMessage(sRKC_Instance[canID].canInstance, &txHeader, txData, &RKC_tx_mailbox) == HAL_OK)
    {
      status = RKC_OK;
    }
  }
  return status;
}

void RKC_Register_RX_Callback(RKC_RX_Callback callback)
{
  if(callback != NULL)
  {
    RKC_callback = callback;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[RKC_DATA_LENGTH];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
  if(RKC_callback != NULL)
  {
    RKC_callback(rxData, (uint8_t)RKC_DATA_LENGTH);
  }
}
