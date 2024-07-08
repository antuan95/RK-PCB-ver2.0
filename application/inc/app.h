/******************************************************************************
 * @file    app.h
 * @brief   Application header file
 *
 * @details This file contains the function declarations and necessary includes
 *          for the application module.
 ******************************************************************************/
#ifndef APP_H_
#define APP_H_

/************ INCLUDES ********************************************************/

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ INTERFACE *******************************************************/
/**
 * @brief Initialize the application.
 *
 * This function initializes the application, setting up any necessary
 * data structures and initial states.
 */
void APP_Init(void);

/**
 * @brief Run the application.
 *
 * This function contains the main loop or execution logic of the application.
 */
void APP_Run(void);

#endif /* APP_H_ */
