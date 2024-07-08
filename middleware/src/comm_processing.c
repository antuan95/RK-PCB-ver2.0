/******************************************************************************
 * @file    comm_processing.c
 * @brief   Communication processing and handling
 * @details This file contains the functions and data structures for managing
 *          communication between various modules, including RFID and main
 *          domains.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "comm_processing.h"
#include "rk_timer.h"
#include "crc.h"
#include "rk_can.h"

/************ LOCAL DEFINES ***************************************************/
#define CP_RESERVED                 0u
#define CP_RFID_POLL_PERIOD         500u
#define CP_RFID_TX_SIZE             4u
#define CP_RFID_REQ_VER_COMMAND     1u
#define CP_RFID_REQ_TAG_COMMAND     2u
/************ LOCAL VARIABLES *************************************************/

/** Data structure for storing RFID data */
CP_data_rfid_TypeDef CP_data_rfid = { 0u };

/** Data structure for storing main data */
CP_data_main_TypeDef CP_data_main = { 0u };

/** Pointer to the main UART domain */
RKU_domain_TypeDef *CP_domain_main;

/** Pointer to the RFID UART domain */
RKU_domain_TypeDef *CP_domain_rfid;

/** Callback function for when RFID data is ready */
CP_Callback_RFID_Data_Ready CP_RFID_Data_Ready_CB = NULL;

CP_Callback_Main_Data_Ready CP_Main_Data_Ready_CB = NULL;

/************ LOCAL FUNCTION DECLARATIONS *************************************/
/** Parse data received in the main domain */
static void CP_Parse_Main_Data(uint8_t *data, uint8_t size);
/** Parse data received in the RFID domain */
static void CP_Parse_RFID_Data(uint8_t *data, uint8_t size);
/** Error handler for communication module */
static void CP_Error_Handler(CP_error_RF_TypeDef er);
/** SysTick callback for RFID processing */
static void CP_Systick_Callback_RFID(void);
/************ LOCAL FUNCTION DEFINITIONS ********************************************/
/**
 * @brief Parse data received in the main domain
 * @param data Pointer to the data buffer
 * @param size Size of the data
 */
static void CP_Parse_Main_Data(uint8_t *data, uint8_t size)
{
    CP_data_main.cmd = data[0u];
    CP_data_main.value = data[1u];
    CP_Main_Data_Ready_CB(&CP_data_main);
}

/**
 * @brief Parse data received from the RFID domain
 * @param data Pointer to the data buffer
 * @param size Size of the data
 */
static void CP_Parse_RFID_Data(uint8_t *data, uint8_t size)
{
    CP_error_RF_TypeDef error = CP_RF_DATA_NO_ERROR;

    // Validate the received data
    if(size != data[1u] + CP_RF_SIZE_OFFSET)
    {
        error = CP_RF_SIZE_ERROR;
    }
    else if(data[0u] != CP_RF_PREAMBLE)
    {
        error = CP_RF_PREAMBLE_ERROR;
    }
    else if((data[2u] != CP_CMD_TAG_UID) && (data[2u] != CP_CMD_VERSION))
    {
        error = CP_RF_COMMAND_ERROR;
    }
    else if(data[size - 1u] != CRC8(data, size - 1u))
    {
        error = CP_RF_CRC_ERROR;
    }
    else
    {
        // Parse valid RFID data
        CP_data_rfid.cmd = data[2u];
        if(CP_data_rfid.cmd == CP_CMD_VERSION)
        {
            for(uint8_t i = 0u; i < CP_TAG_VER_SIZE; i++)
            {
                CP_data_rfid.tag_version[i] = data[i + CP_RFID_PAYLOAD_VERSION_OFFSET];
            }
        }
        else if(CP_data_rfid.cmd == CP_CMD_TAG_UID)
        {
            CP_data_rfid.tag_uid = data[3u];
            CP_data_rfid.value_sensors = data[4u];
        }
        // Trigger the RFID data ready callback
        CP_RFID_Data_Ready_CB(&CP_data_rfid);
    }
    if(error != CP_RF_DATA_NO_ERROR)
    {
      CP_Error_Handler(error);
    }

    // Wait for the next data
    RKU_Wait_For_Data(CP_domain_rfid);
}

static void CP_Systick_Callback_RFID(void)
{
    CP_Send_Request_RF_Tag(CP_domain_rfid);
}

static void CP_Error_Handler(CP_error_RF_TypeDef er)
{
  //TODO Error Handler
}

/************ PUBLIC FUNCTION DEFINITIONS ********************************************/

void CP_Init(void)
{
    // Initialize RFID domain and send request for RFID version

    CP_domain_rfid = RKU_Get_Domain(RKU_RFID);
    CP_Send_Request_RF_Version(CP_domain_rfid);
    RKU_Wait_For_Data(CP_domain_rfid);
    // Register callback functions
    TM_timer_instance_TypeDef timer_instance =
    {
        .period = CP_RFID_POLL_PERIOD,
        .type = TM_TICK_1MS,
        .callback = CP_Systick_Callback_RFID
    };
    TM_Init_Instance_Free_Running(&timer_instance);
    RKU_Register_RX_Callback(RKU_RFID, CP_Parse_RFID_Data);
    RKC_Register_RX_Callback(CP_Parse_Main_Data);
}

void CP_Register_RFID_Data_Ready_Callback(CP_Callback_RFID_Data_Ready callback)
{
    if(callback != NULL)
    {
        CP_RFID_Data_Ready_CB = callback;
    }
}

void CP_Register_Main_Data_Ready_Callback(CP_Callback_Main_Data_Ready callback)
{
  if(callback != NULL)
  {
      CP_Main_Data_Ready_CB = callback;
  }
}

void CP_Send_Request_RF_Version(RKU_domain_TypeDef *domain)
{
    domain->data_tx[0u] = CP_RF_PREAMBLE;
    domain->data_tx[1u] = CP_RF_TX_PAYLOAD_SIZE_VERSION;
    domain->data_tx[2u] = CP_RFID_REQ_VER_COMMAND;
    domain->data_tx[3u] = CRC8(domain->data_tx, CP_RF_TX_PAYLOAD_SIZE_VERSION + CP_RF_SIZE_OFFSET - 1u);
    domain->tx_size = CP_RFID_TX_SIZE;
    RKU_Send_Data(domain);
}

void CP_Send_Request_RF_Tag(RKU_domain_TypeDef *domain)
{
    domain->data_tx[0u] = CP_RF_PREAMBLE;
    domain->data_tx[1u] = CP_RF_TX_PAYLOAD_SIZE_TAG;
    domain->data_tx[2u] = CP_RFID_REQ_TAG_COMMAND; // RFID Tag
    domain->data_tx[3u] = CRC8(domain->data_tx, CP_RF_TX_PAYLOAD_SIZE_TAG + CP_RF_SIZE_OFFSET - 1u);
    domain->tx_size = CP_RFID_TX_SIZE;
    RKU_Send_Data(domain);
}

void CP_Send_Data_To_Main(CP_response_main_TypeDef response)
{
  uint8_t response_to_send[RKC_DATA_LENGTH] = {0};
  response_to_send[0] = CP_RESERVED;
  response_to_send[1] = response.tag;
  response_to_send[2] = CP_RESERVED;
  response_to_send[3] = response.sensors;
  response_to_send[4] = response.motor_position;
  response_to_send[5] = response.arm_position;
  response_to_send[6] = response.enc_button_counter;
  response_to_send[7] = response.enc_value;
  RKC_Add_Tx_Message(RKC_MAIN, response_to_send);
}
