/*
 * config.h
 *
 *  Created on: Oct 24, 2024
 *      Author: jmorritt
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define FIRMWARE_VERSION 1.0

#define STM32L4XX
#define MAIN_HEADER "BoomLED.hpp"

#define STM_HAL "stm32l4xx_hal.h"
#define STM_CAN "can.h"
#define STM_HAL_IWDG "stm32l4xx_hal_iwdg.h"
#define STM_HAL_FLASH "stm32l4xx_hal_flash.h"

// Set up all our definitions
// Enable libcanard
#define LIBCANARD_ENABLED
#define LIBCANARD_MESSAGE_NODE
#define LIBCANARD_MESSAGE_PARAMETERS
#define LIBCANARD_MESSAGE_ARRAYCOMMAND
#define LIBCANARD_MESSAGE_LIGHTSCOMMAND
#define LIBCANARD_MESSAGE_NOTIFYSTATE
#define LIBCANARD_MESSAGE_ARMINGSTATUS

#define LED_ERROR1_PIN GPIO_PIN_7
#define LED_ERROR1_GPIO_PORT GPIOB

#define ERROR1_GPIO_Port LED_ERROR1_GPIO_PORT
#define ERROR1_Pin LED_ERROR1_PIN

#define UART1_ENABLED

#endif /* INC_CONFIG_H_ */
