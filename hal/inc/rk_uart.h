/******************************************************************************
 * @file    rk_uart.h
 * @brief   UART communication handling for multiple domains
 * @details This file contains the definitions and function prototypes for
 *          managing UART communication across different domains, including
 *          functions for data transmission, reception, and direction switching.
 ******************************************************************************/

#ifndef _RK_UART_H
#define _RK_UART_H

/************ INCLUDES ********************************************************/
#include <stdint.h>
#include <stdbool.h>
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
/** Maximum length of UART packet */
#define RKU_DATA_MAX_SIZE   128u
#define USE_FULL_ASSERT     ///< Macro to enable assert in debug mode

/************ TYPE DEFS *******************************************************/
/** Data readiness status */
typedef enum
{
    RKU_DATA_NOT_READY = 0u, /**< Data not ready */
    RKU_DATA_READY         /**< Data ready */
} RKU_dataReady_TypeDef;

/** Domain IDs for UART communication */
typedef enum
{
    RKU_MAIN = 0u,       /**< Main domain */
    RKU_RFID,            /**< RFID domain */
    RKU_DOMAINS_NUMBER   /**< Number of available UARTs */
} RKU_domainID_TypeDef;

/** RE/DE pin configuration status */
typedef enum
{
    RKU_RE_DE_OFF, /**< RE/DE pin off */
    RKU_RE_DE_ON   /**< RE/DE pin on */
} RKU_re_de_TypeDef;

/** Direction of UART communication */
typedef enum
{
    RKU_RX = 0u, /**< Receiving data */
    RKU_TX       /**< Transmitting data */
} RKU_direction_TypeDef;

/** RX Callback */
typedef void(*RKU_RX_Callback)(uint8_t *data, uint8_t size);
typedef void(*RKU_TX_Callback)(void);
typedef void(*RKU_Error_Callback)(void);

/** Structure to hold RE/DE pin configuration for half-duplex communication */
typedef struct
{
    RKU_re_de_TypeDef is_half_duplex; /**< Half-duplex configuration status */
    GPIO_TypeDef * re_de_port;        /**< Port for RE/DE pin */
    uint16_t re_de_pin;               /**< Pin number for RE/DE */
} RKU_direction_switch_TypeDef;

/** Structure to manage UART domain data */
typedef struct
{
    uint8_t rx_size;                                /**< Size of received data */
    uint8_t tx_size;                                /**< Size of data to be transmitted */
    uint8_t data_tx[RKU_DATA_MAX_SIZE];             /**< Buffer for data to be transmitted */
    uint8_t data_rx[RKU_DATA_MAX_SIZE];             /**< Buffer for received data */
    UART_HandleTypeDef *uart;                       /**< UART handle */
    RKU_dataReady_TypeDef ready;                    /**< Data readiness status */
    RKU_direction_switch_TypeDef direction_switch;  /**< Direction switch configuration */
    RKU_domainID_TypeDef domainID;                  /**< Domain ID */
    RKU_RX_Callback rx_callback;                    /**< RX callback function */
    RKU_TX_Callback tx_callback;                    /**< TX callback function */
    RKU_Error_Callback error_callback;              /**< Error callback function */
} RKU_domain_TypeDef;

/************ FUNCTION PROTOTYPES *********************************************/
/**
 * @brief Initialize a UART instance for a domain
 *
 * @param domainID ID of the domain
 * @param huart Pointer to the UART handle
 * @return Pointer to the initialized domain
 */
RKU_domain_TypeDef* RKU_Set_Uart_Instance(RKU_domainID_TypeDef domainID, UART_HandleTypeDef *huart);

/**
 * @brief Check if data is ready in the specified domain
 *
 * @param domain Pointer to the domain
 * @return Data readiness status
 */
RKU_dataReady_TypeDef RKU_Is_Data_Ready(RKU_domain_TypeDef *domain);

/**
 * @brief Wait for data to be received in the specified domain
 *
 * @param domain Pointer to the domain
 */
void RKU_Wait_For_Data(RKU_domain_TypeDef *domain);

/**
 * @brief Send data stored in the specified domain's buffer
 *
 * @param domain Pointer to the domain
 */
void RKU_Send_Data(RKU_domain_TypeDef *domain);

/**
 * @brief Enable direction switch for half-duplex communication
 *
 * @param domain Pointer to the domain
 * @param GPIOx Port for the RE/DE pin
 * @param GPIO_Pin Pin number for the RE/DE pin
 */
void RKU_Enable_Direction_Switch(RKU_domain_TypeDef *domain, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * @brief Append a byte to the transmission buffer of the specified domain
 *
 * @param domain Pointer to the domain
 * @param byte Byte to be appended
 */
void RKU_Append_Byte(RKU_domain_TypeDef *domain, uint8_t byte);

/**
 * @brief Append an array of bytes to the transmission buffer of the specified domain
 *
 * @param domain Pointer to the domain
 * @param array Pointer to the array of bytes to be appended
 * @param size Size of the array
 */
void RKU_Append_Array(RKU_domain_TypeDef *domain, uint8_t *array, uint8_t size);

/**
 * @brief Check if there is data to send in the specified domain
 *
 * @param domain Pointer to the domain
 * @return Size of the data to be sent
 */
uint8_t RKU_Is_Data_To_Send(RKU_domain_TypeDef *domain);

/**
 * @brief Set the domain ID for the specified domain
 *
 * @param domain Pointer to the domain
 * @param domainID Domain ID to be set
 */
void RKU_Set_DomainID(RKU_domain_TypeDef *domain, RKU_domainID_TypeDef domainID);

/**
 * @brief Get the domain structure for the specified domain ID
 *
 * @param domainID Domain ID
 * @return Pointer to the domain structure
 */
RKU_domain_TypeDef * RKU_Get_Domain(RKU_domainID_TypeDef domainID);

/**
 * @brief Register an RX callback function for the specified domain ID
 *
 * @param domainID Domain ID
 * @param callback RX callback function to register
 */
void RKU_Register_RX_Callback(RKU_domainID_TypeDef domainID, RKU_RX_Callback callback);

#endif /* _RK_UART_H */
