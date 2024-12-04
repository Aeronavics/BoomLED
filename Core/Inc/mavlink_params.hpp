/*
 * mavlink_params.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: jmorritt
 */

#ifndef INC_MAVLINK_PARAMS_HPP_
#define INC_MAVLINK_PARAMS_HPP_

// INCLUDE REQUIRED HEADER FILES FOR INTERFACE.
#include "params.hpp"
#include <stdio.h>
// DECLARE PUBLIC GLOBAL VARIABLES.

// TODO - This file should be generated at compiletime from a PRM file.

// NOTE - The order of the parameters and their indices has to be the same, or it all goes horribly wrong.

enum PARAM_INDEX {
    PARAM_IDX_SID,
    PARAM_IDX_CID,
};

// TODO - Non-volatile storage of parameters is affected by writing the entire array into EEPROM: this isn't exactly space efficient, and probably needs to be changed at some stage.

#ifndef EXTERN  // Fairly common C/CPP magic to allow global defintions in header files: in this case, the matching EXTERN is in params.cpp.
extern const uint8_t PARAM_COMPAT_VERSION;
extern Param parameters[];
extern const size_t NUM_PARAMS;
#else
const uint8_t PARAM_COMPAT_VERSION = 0x0A; // EVERY TIME YOU CHANGE THE PARAMETER DEFINITIONS, INCREMENT THIS NUMBER.
Param parameters[] = {
    Param(Param_name("SID"), Param_value::from<uint8_t>(1)),
    Param(Param_name("CID"), Param_value::from<uint8_t>(MAV_COMP_ID_IPDB))
};
const size_t NUM_PARAMS = sizeof (parameters) / sizeof (Param);
#endif

#endif /* INC_MAVLINK_PARAMS_HPP_ */
