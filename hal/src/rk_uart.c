/******************************************************************************
 * @file    rk_uart.c
 * @brief   UART communication management
 * @details This file contains the implementation of functions for handling
 *          UART communication, including data transmission, reception, and
 *          direction control in half-duplex mode.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_uart.h"

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ LOCAL DATA ******************************************************/
/** Array to hold domain instances for UART communication */
RKU_domain_TypeDef RKU_domains[RKU_DOMAINS_NUMBER] = {0u};
/** Counter to keep track of the number of UART instances */
uint8_t RKU_uart_number = 0u;

/************ LOCAL FUNCTION DECLARATION **************************************/
/**
 * @brief Switch the direction of UART communication
 * @param domain Pointer to the UART domain
 * @param dir Direction to switch to (TX or RX)
 */
static void RKU_Switch_Direction(RKU_domain_TypeDef *domain, RKU_direction_TypeDef dir);
/**
 * @brief Reset the UART transmission buffer
 * @param domain Pointer to the UART domain
 */
static void RKU_Reset_TX_Buf(RKU_domain_TypeDef *domain);

/************ LOCAL FUNCTION DEFINITION ***************************************/
static void RKU_Switch_Direction(RKU_domain_TypeDef *domain, RKU_direction_TypeDef dir)
{
    if(domain->direction_switch.is_half_duplex == RKU_RE_DE_ON)
    {
        switch(dir)
        {
        case RKU_TX:
            HAL_GPIO_WritePin(domain->direction_switch.re_de_port,
                              domain->direction_switch.re_de_pin, GPIO_PIN_SET);
            break;
        case RKU_RX:
            HAL_GPIO_WritePin(domain->direction_switch.re_de_port,
                              domain->direction_switch.re_de_pin, GPIO_PIN_RESET);
            break;
        default:
            break;
        }
    }
}

static void RKU_Reset_TX_Buf(RKU_domain_TypeDef *domain)
{
    domain->tx_size = 0u;
}

/************ PUBLIC FUNCTION DEFINITION **************************************/
RKU_domain_TypeDef* RKU_Set_Uart_Instance(RKU_domainID_TypeDef domainID, UART_HandleTypeDef *huart)
{
  if((huart != NULL) && (domainID < RKU_DOMAINS_NUMBER))
  {
    RKU_domains[domainID].uart = huart;
    RKU_domains[domainID].domainID = domainID;
  }
    return &RKU_domains[RKU_uart_number++];
}

void RKU_Register_RX_Callback(RKU_domainID_TypeDef domainID, RKU_RX_Callback callback)
{
  if(callback != NULL)
  {
    for(uint8_t i = 0u; i < RKU_DOMAINS_NUMBER; i++)
    {
      if(RKU_domains[i].domainID == domainID)
      {
        RKU_domains[i].rx_callback = callback;
      }
    }
  }
}

RKU_dataReady_TypeDef RKU_Is_Data_Ready(RKU_domain_TypeDef *domain)
{
    return domain->ready;
}

void RKU_Append_Byte(RKU_domain_TypeDef *domain, uint8_t byte)
{
    if(domain->tx_size < RKU_DATA_MAX_SIZE)
    {
        domain->data_tx[domain->tx_size++] = byte;
    }
    else
    {
        // TO DO: Buffer is full
    }
}

void RKU_Append_Array(RKU_domain_TypeDef *domain, uint8_t *array, uint8_t size)
{
    for (uint8_t i = 0u; i < size; i++)
    {
        RKU_Append_Byte(domain, array[i]);
    }
}

uint8_t RKU_Is_Data_To_Send(RKU_domain_TypeDef *domain)
{
    return domain->tx_size;
}

void RKU_Wait_For_Data(RKU_domain_TypeDef *domain)
{
    if(domain->uart != NULL)
    {
        domain->rx_size = 0u;
        domain->ready = RKU_DATA_NOT_READY;
        HAL_UARTEx_ReceiveToIdle_IT(domain->uart,
                                    domain->data_rx,
                                    RKU_DATA_MAX_SIZE);
    }
    else
    {
        // TO DO: Handle error
    }
}

void RKU_Send_Data(RKU_domain_TypeDef *domain)
{
    if(domain->uart != NULL)
    {
        RKU_Switch_Direction(domain, RKU_TX);
        HAL_UART_Transmit_IT(domain->uart,
                             domain->data_tx,
                             domain->tx_size);
    }
    else
    {
        // TO DO: Handle error
    }
}

void RKU_Enable_Direction_Switch(RKU_domain_TypeDef *domain, GPIO_TypeDef* port, uint16_t pin)
{
    if(domain->uart != NULL)
    {
        domain->direction_switch.is_half_duplex = RKU_RE_DE_ON;
        domain->direction_switch.re_de_port = port;
        domain->direction_switch.re_de_pin = pin;
    }
    else
    {
        // TO DO: Handle error
    }
}

void RKU_Set_DomainID(RKU_domain_TypeDef *domain, RKU_domainID_TypeDef domainID)
{
    domain->domainID = domainID;
}

RKU_domain_TypeDef * RKU_Get_Domain(RKU_domainID_TypeDef domainID)
{
    return &RKU_domains[domainID];
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    for(uint8_t i = 0u; i < RKU_DOMAINS_NUMBER; i++)
    {
        if(huart == RKU_domains[i].uart)
        {
            RKU_domains[i].rx_size = size;
            RKU_domains[i].ready = RKU_DATA_READY;
            if(RKU_domains[i].rx_callback != NULL)
            {
              RKU_domains[i].rx_callback(RKU_domains[i].data_rx, RKU_domains[i].rx_size);
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0u; i < RKU_DOMAINS_NUMBER; i++)
    {
        if(huart == RKU_domains[i].uart)
        {
            RKU_Switch_Direction(&RKU_domains[i], RKU_RX);
            RKU_Reset_TX_Buf(&RKU_domains[i]);
            if(RKU_domains[i].tx_callback != NULL)
            {
              RKU_domains[i].tx_callback();
            }
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0u; i < RKU_DOMAINS_NUMBER; i++)
    {
        if(huart == RKU_domains[i].uart)
        {
            RKU_domains[i].ready = RKU_DATA_NOT_READY;
            HAL_UARTEx_ReceiveToIdle_IT(huart, RKU_domains[i].data_rx, RKU_DATA_MAX_SIZE);
            if(RKU_domains[i].error_callback != NULL)
            {
              RKU_domains[i].error_callback();
            }
        }
    }
}
