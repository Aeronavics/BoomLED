/*
 * can_params.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: jmorritt
 */

//#ifndef INC_CAN_PARAMS_HPP_
//#define INC_CAN_PARAMS_HPP_

#pragma once


// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.
#include "libcanard_module.hpp"
#include <stdio.h>
#include <uavcan.protocol.param.GetSet_res.h>
// DECLARE PUBLIC GLOBAL VARIABLES.
// NOTE - The order of the parameters and their indices has to be the same, or it all goes horribly wrong.



enum CAN_PARAM_INDEX {
        CAN_PARAM_IDX_CAN_ID, //This is is a requirement!
        CAN_PARAM_IDX_BOOM_ID,
		CAN_PARAM_IDX_LEDSTAT_SRC_ID, //sets the source channel to listen to for a broadcast lights message
		CAN_PARAM_IDX_LEDSTAT_BR,   //LED Brightness for flight controller status indication
		CAN_PARAM_IDX_LEDNAV_BR,    //LED Brightness when flashing nav lights
		CAN_PARAM_IDX_LED_CHANNEL, //use this channel to control the LED mode.
};

// TODO - Non-volatile storage of parameters is affected by writing the entire array into EEPROM: this isn't exactly space efficient, and probably needs to be changed at some stage.

#ifndef CAN_EXTERN  // Fairly common C/CPP magic to allow global definitions in header files: in this case, the matching EXTERN is in params.cpp.
extern const uint8_t CAN_PARAM_COMPAT_VERSION;
extern uavcan_protocol_param_GetSetResponse can_parameters[];
extern const size_t NUM_CAN_PARAMS;
#else
const uint8_t CAN_PARAM_COMPAT_VERSION = 0x01; // EVERY TIME YOU CHANGE THE PARAMETER DEFINITIONS, INCREMENT THIS NUMBER.
uavcan_protocol_param_GetSetResponse can_parameters[] =
{
        new_integer_can_param("NodeID",                 35,     35),
        new_integer_can_param("BoomID",								0,			0),
		new_integer_can_param("LED_SourceNodeID", 1, 1, 0, 127), //default listen to the flight controller
		new_integer_can_param("LED_StatusBrightness", 100, 100, 0, 1000),
		new_integer_can_param("LED_NavBrightness", 1000, 1000, 0, 1000),
		new_integer_can_param("LED_CtrlChan", 15, 15, 0, 255),

};
const size_t NUM_CAN_PARAMS = (sizeof (can_parameters) / sizeof (uavcan_protocol_param_GetSetResponse))+1;

#endif


//#endif /* INC_CAN_PARAMS_HPP_ */
