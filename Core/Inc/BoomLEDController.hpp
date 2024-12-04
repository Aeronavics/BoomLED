/*
 * LoadcellController.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: jmorritt
 */

#ifndef INC_BOOMLEDCONTROLLER_HPP_
#define INC_BOOMLEDCONTROLLER_HPP_

// Include all our modules
// Include driver modules for peripherals.
#include "mavlink.hpp"
#include "driver_module.hpp"
#include "libcanard_module.hpp"
#include "params.hpp"
#include "uart_module.hpp"

#include "led_driver.hpp"

#include <uavcan.protocol.file.BeginFirmwareUpdate_req.h>

#include "main.h"
#include "config.h"

#include "adc.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"

#include "stm32l4xx_hal.h"

//details shown in CAN Node info
#define UAVCAN_MODULE_NAME "com.aeronavics.boomLED"


#endif /* INC_BOOMLEDCONTROLLER_HPP_ */
