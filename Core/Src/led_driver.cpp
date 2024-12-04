/*
 * led_driver.cpp
 *
 *  Created on: Oct 25, 2024
 *      Author: jmorritt
 */

#include "led_driver.hpp"

/**
 * @brief  Creates and returns a singleton instance of the Led driver
 * @retval Singleton instance of the Led driver
 */
Led_driver& Led_driver::get_driver(void)
{
	// Create a singleton for the driver.
	static Led_driver singleton;

	// Return the driver.
	return singleton;
}

/**
 * @brief  Led driver deconstructor
 * @retval None
 */
Led_driver::~Led_driver(void)
{
	return;
}

/**
 * @brief  Handles Led drivers unthrottled loop
 * @retval None
 */
void Led_driver::sync_update_unthrottled(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	return;
}

/**
 * @brief  Handles Led drivers 100Hz loop
 * @retval None
 */
void Led_driver::sync_update_100Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}

	set_led_output();

	update_LEDs();

	return;
}

/**
 * @brief  Handles Led drivers 10Hz loop
 * @note   Transmits Led status telemetry in this loop.
 * @retval None
 */
void Led_driver::sync_update_10Hz(void)
{
	// If we're not in STATE_NORMAL, then we don't do anything yet.
	if (this_state != DRIVER_STATE_NORMAL)
	{
		return;
	}
	
	//reset override if timed out
    if(HAL_GetTick()-led_override.last_override_time > OVERRIDE_TIMEOUT)
	{
      led_override.is_active = false;
      led_override.source_id = 0;  
      led_override.priority = PRIORITY_NONE;
      led_override.type = OVERRIDE_NONE;           
    } 

	// Calculate temperature moving averages
	calculate_temperature();

	// Transmit boom telemetry
	transmit_telemetry();
	return;
}

/**
 * @brief  Handles Led drivers 1Hz loop
 * @note   1Hz loop handles all housekeeping.
 * @retval None
 */
void Led_driver::sync_update_1Hz(void)
{
	// Run the driver housekeeping state machine.

	// If we don't specify a new state, assume we will just remain in the current state.
	Driver_state next_state = this_state;

	// Select behaviour depending on the current state.
	switch (this_state)
	{
		case DRIVER_STATE_UNKNOWN:
		{
			// If we don't know which state we are in, then we probably want to initialise.
			next_state = DRIVER_STATE_INIT;
			break;
		}
		case DRIVER_STATE_INIT:
		{
			MX_DMA_Init();
			MX_ADC1_Init();
			// Intialise ADCs
			if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
			{
				// Calibration Error
				Error_Handler();
			}

			if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&temperature_adc_reading, 1) != HAL_OK)
			{
				// Start Error
				Error_Handler();
			}

			// Intialise timer
			MX_TIM2_Init();
			// Set pwm to 0
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, led.red);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, led.green);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, led.blue);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, led.white);
			// Start pwm on each channel 
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // RED 
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // GREEN
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // BLUE
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // WHITE

			loopTimer = HAL_GetTick();

			Libcanard_module::get_driver().sendLog(impl_::LogLevel::Debug, "LED Driver initialized");
			next_state = DRIVER_STATE_NORMAL;
			break;
		}
		case DRIVER_STATE_NORMAL:
		{
			break;
		}
		case DRIVER_STATE_ERROR:
		{
			// We'll attempt to reinitialise; that's about all we can hope for.
			next_state = DRIVER_STATE_INIT;
			break;
		}
		default:
		{
			// We shouldn't ever end up here.
			this_state = DRIVER_STATE_UNKNOWN;
			next_state = DRIVER_STATE_UNKNOWN;
			break;
		}
	}

	// Advance to the next state.
	prev_state = this_state;
	this_state = next_state;

	// All done.

	return;
}

/**
 * @brief  Calculates the LED colour.
 * @note   Decides what mode the LEDs should should be in at the time
 * 			and based on that what colours the LEDs should be.
 * @retval None
 */
void Led_driver::set_led_output(){
	uint16_t nav_brightness = get_can_param_by_id(CAN_PARAM_IDX_LEDNAV_BR)->value.integer_value;

	uint8_t boom_id = get_can_param_by_id(CAN_PARAM_IDX_BOOM_ID)->value.integer_value;

	if(isSystemRunning){

		if (boom_id == 0)
		{
			mode = LED_MODE_RED_FADE; // Boom can params are not set up
		}
		//overrides to normal operation
		else if(HAL_GetTick()-last_can_received_time > 100)
		{
			mode = LED_MODE_STROBE; //oh dear, we've lost the flight controller
		}

		else if(led_override.is_active)
		{
			mode = LED_MODE_OVERRIDE;
		}

		else if(isSystemArmed)
		{
			switch(can_led_mode)
			{ // three options for flash pattern when in the air, based on switch position
				case 0:
				{
					mode = LED_MODE_STATUS;
					break;
				}
				case 1:
				{
					mode = LED_MODE_NAV;
					break;
				}
				case 2:
				{
					mode = LED_MODE_STROBE;
					break;
				}
			}
		}
		else
		{
			mode = LED_MODE_STATUS; //always have status lights active when disarmed
		}
	}
	else{
		mode = LED_MODE_FADE; // have not received can data for the first time yet
	}

	switch(mode)
	{
		case LED_MODE_OFF:
		{
			setLED(0, 0, 0, 0);
			break;
		}
		case LED_MODE_RED_FADE:
		{
			uint16_t fade_output = (FADE_MAX_BRIGHTNESS / 2) * sin(HAL_GetTick() * (M_2PI / FADE_PERIOD) - M_PI) + (FADE_MAX_BRIGHTNESS / 2);
			setLED(fade_output, 0, 0, 0);
			break;
		}
		case LED_MODE_FADE:
		{
			uint16_t fade_output = (FADE_MAX_BRIGHTNESS / 2) * sin(HAL_GetTick() * (M_2PI / FADE_PERIOD) - M_PI) + (FADE_MAX_BRIGHTNESS / 2);
			setLED(fade_output, fade_output, fade_output, 0);
			break;
		}
		case LED_MODE_OVERRIDE:
		{
			uint16_t red = map_LED(led_override.colour.red, 0, 1000, 0, nav_brightness);
			uint16_t green = map_LED(led_override.colour.green, 0, 1000, 0, nav_brightness);
			uint16_t blue = map_LED(led_override.colour.blue, 0, 1000, 0, nav_brightness);

			setLED(red, green, blue, 0);
			break;
		}
		case LED_MODE_STATUS:
		{
			//set all LEDs to flight controller status colour
			setLED(status_colour.red / 2, status_colour.green / 2, status_colour.blue / 2, 0);
			break;
		}
		case LED_MODE_STROBE:
		{
			if((FLASH_PATTERN_STROBE >> flash_pattern_index) & 0x01){
				setLED(LED_PWM_MAX_PERIOD_RELOAD, LED_PWM_MAX_PERIOD_RELOAD, LED_PWM_MAX_PERIOD_RELOAD, LED_PWM_MAX_PERIOD_RELOAD);
			}
			else
			{
				if (boom_id == 1)
				{
					setLED(0, nav_brightness, 0, 0);
				}
				else if (boom_id == 3)
				{
					setLED(nav_brightness, 0, 0, 0);
				}
				else if (boom_id == 2 || boom_id == 4) 
				{
					setLED(0, 0, 0, nav_brightness);
				}
			}
			break;
		}
		case LED_MODE_NAV:
		{
			if (boom_id == 1)
			{
				if((FLASH_PATTERN_PORT >> flash_pattern_index) & 0x01)
				{
					setLED(0, nav_brightness, 0, 0);
				}
				else
				{
					setLED(0, 0, 0, 0);
				}
			}
			else if (boom_id == 3)
			{
				if((FLASH_PATTERN_STARBOARD >> flash_pattern_index) & 0x01)
				{
					setLED(nav_brightness, 0, 0, 0);
				}
				else
				{
					setLED(0, 0, 0, 0);
				}
			}
			else if (boom_id == 2)
			{
				if((FLASH_PATTERN_STARBOARD >> flash_pattern_index) & 0x01)
				{
					setLED(0, 0, 0, nav_brightness);
				}
				else
				{
					setLED(0, 0, 0, 0);
				}
			}
			else if (boom_id == 4)
			{
				if((FLASH_PATTERN_PORT >> flash_pattern_index) & 0x01)
				{
					setLED(0, 0, 0, nav_brightness);
				}
				else
				{
					setLED(0, 0, 0, 0);
				}
			}
			break;
		}
	}

	if(HAL_GetTick() - loopTimer > 100)
	{
		loopTimer = HAL_GetTick();
		if (flash_pattern_index == 0 || flash_pattern_index > 32)
		{
			flash_pattern_index = 32;
		}
		flash_pattern_index--;
	}
}

/**
 * @brief  Updates the pwm for each of the colour channels.
 * @retval None
 */
void Led_driver::update_LEDs()
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, led.red);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, led.green);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, led.blue);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, led.white);
}


/**
 * @brief  	Handles incoming CAN messages
 * @param  	transfer : The CAN message
 * @param  	data_type_signature : CAN signature of the transfer
 * @param	data_type_id : CAN ID of the transfer
 * @param	inout_transfer_id : transfer id of the transfer
 * @param	priority : Priority of the transfer
 * @param  	payload : CAN payload of the transfer
 * @param	payload_len : Length of the CAN payload
 * @note   	Listens to the incoming can messages and processes them if required.
 * @retval 	None
 */
void Led_driver::handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len)
{
	switch (transfer->data_type_id)
	{
	case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
	{
		//This is a broadcast message
		//We check to see if we actually care about the source.

		uavcan_equipment_indication_LightsCommand lights_command;
		//decode the actual transfer.
		uavcan_equipment_indication_LightsCommand_decode(transfer, &lights_command);

		uint16_t max_brightness = get_can_param_by_id(CAN_PARAM_IDX_LEDSTAT_BR)->value.integer_value;

		for(uint8_t i = 0; i < lights_command.commands.len; i++)
		{
			//check if we are interpreting this as a status light ID, this is an RGB LED
			if(lights_command.commands.data[i].light_id == LIGHTS_COMMAND_STATUS_LIGHT_ID)
			{
				if(transfer->source_node_id == 1) //lights message coming from flight controller
				{
					if(!isSystemRunning && (lights_command.commands.data[i].color.red != 0 || lights_command.commands.data[i].color.green != 0 || lights_command.commands.data[i].color.blue != 0))
					{
						isSystemRunning = true; //take us out of fade startup mode when FC starts outputting lights messages
					}
					//set the status lights, by mapping them to the relevant values.
					status_colour.red   = map_LED(lights_command.commands.data[i].color.red, 0, 31, 0, max_brightness);
					status_colour.green = map_LED(lights_command.commands.data[i].color.green, 0, 63, 0, max_brightness);
					status_colour.blue  = map_LED(lights_command.commands.data[i].color.blue, 0, 31, 0, max_brightness);
					status_colour.white = 0;
				}
				else if(!led_override.is_active || (led_override.type == OVERRIDE_CAN && transfer->source_node_id == led_override.source_id) || led_override.priority < PRIORITY_HIGH){ //lights message coming from anywhere else
					led_override.source_id = transfer->source_node_id;
					led_override.is_active = true;
					led_override.last_override_time = HAL_GetTick();
					led_override.type = OVERRIDE_CAN;
					led_override.colour.red   = map_LED(lights_command.commands.data[i].color.red, 0, 31, 0, max_brightness);
					led_override.colour.green = map_LED(lights_command.commands.data[i].color.green, 0, 63, 0, max_brightness);
					led_override.colour.blue  = map_LED(lights_command.commands.data[i].color.blue, 0, 31, 0, max_brightness);
					led_override.colour.white = 0;
					led_override.priority = PRIORITY_HIGH;
				}
			}
		}
		break;
	}
	case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
	{
		uavcan_equipment_actuator_ArrayCommand array_command;

		uavcan_equipment_actuator_ArrayCommand_decode(transfer, &array_command);

		//Get the channel we are interested in:
		uint8_t led_channel = (uint8_t)get_can_param_by_id(CAN_PARAM_IDX_LED_CHANNEL)->value.integer_value;

		//loop through what we have received and do something
		for(uint8_t i = 0; i < array_command.commands.len; i++)
		{
			//check we have the correct channel and the correct unit type. UNITLESS is for standard cube PWM's
			if(array_command.commands.data[i].actuator_id == led_channel && array_command.commands.data[i].command_type == UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS)
			{
				float val = array_command.commands.data[i].command_value;
				//Set the LED's brightness based on what we have received.

				if (val <= -0.25)
				{
					can_led_mode = 0;
				}
				else if ((val > -0.25) && (val <= 0.25))
				{
					can_led_mode = 1;
				}
				else if (val > 0.25)
				{
					can_led_mode = 2;
				}
			}
		}
		break;
	}
	case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
	{
		uavcan_equipment_safety_ArmingStatus armingStatus;
		uavcan_equipment_safety_ArmingStatus_decode(transfer, &armingStatus);

		if(armingStatus.status>125)
		{
			isSystemArmed = true;
		}
		else
		{
			isSystemArmed = false;
		}
		break;
	}
	case ARDUPILOT_INDICATION_NOTIFYSTATE_ID:
	{
		last_can_received_time = HAL_GetTick();
	}
	default:
	{
		break;
	}
	}
}

/**
 * @brief  Transmits Led driver telemetry
 * @note   Transmits a com_aeronavics_BoomStatus msg over CAN.
 * @retval None
 */
void Led_driver::transmit_telemetry(void)
{
	uint8_t transfer_id = 0;
	uint32_t length = 0;

	com_aeronavics_BoomStatus boom_info;

	boom_info.boom_id = get_can_param_by_id(CAN_PARAM_IDX_BOOM_ID)->value.integer_value;

	boom_info.connection_status = HAL_GPIO_ReadPin(BOOM_DETECT_GPIO_Port, BOOM_DETECT_Pin);

	boom_info.temperature = measured_temp;

	uint8_t boom_info_buffer[COM_AERONAVICS_BOOMSTATUS_MAX_SIZE];
	length = com_aeronavics_BoomStatus_encode(&boom_info, boom_info_buffer);
	driverhost_broadcast_can(
			nullptr,
			COM_AERONAVICS_BOOMSTATUS_SIGNATURE,
			COM_AERONAVICS_BOOMSTATUS_ID,
			&transfer_id,
			CAN_TRANSFER_PRIORITY_MEDIUM,
			&boom_info_buffer,
			length,
			this
	);

}

/**
 * @brief  Calculates the temperature measured on the LED Board.
 * @retval None
 */
void Led_driver::calculate_temperature(void)
{
	// get moving average of ADC readings
	float average_adc = calculate_average(temperature_sensor.raw_values, TELEM_MOVING_AVERAGE_BINS);

	float calculated_voltage =  (average_adc - TEMP_ADC_MIN) * (ADC_VOLT_MAX - ADC_VOLT_MIN) / (TEMP_ADC_MAX - TEMP_ADC_MIN) + ADC_VOLT_MIN;

	float calculated_resistance = (10000 * calculated_voltage) / (ADC_VOLT_MAX - calculated_voltage);

	measured_temp = (-(TEMP_RESISTOR_T1) / (((log(TEMP_RESISTOR_R1 / calculated_resistance) / TEMP_RESISTOR_B_CONSTANT) * TEMP_RESISTOR_T1) - 1)) - KELVIN_OFFSET;
}

/**
 * @brief  Calculates the average of a given array
 * @param  array : Array to calculate average over
 * @param  size	: Size of the array
 * @retval float average of the array
 */
float Led_driver::calculate_average(volatile uint32_t* array, uint8_t size)
{
	float temp_value = 0;
	for(uint8_t i = 0; i < size; i++){
		temp_value += array[i];
	}
	return temp_value / size;
}

/**
 * @brief  Maps and sets the colours for the LEDs.
 * @param  red : 0-1000 of intensity of red.
 * @param  green : 0-1000 of intensity of green.
 * @param  blue : 0-1000 of intensity of blue.
 * @param  white : 0-1000 of intensity of white.
 * @retval None.
 */
void Led_driver::setLED(uint16_t red, uint16_t green, uint16_t blue, uint16_t white){
	if(red > 1000)red = 1000;
	if(green > 1000)green = 1000;
	if(blue > 1000) blue = 1000;
	if(white > 1000) white = 1000;

	led.red = map_LED(red,0,LED_PSEUDO_SCALE_MAX,0,LED_PWM_MAX_PERIOD_RELOAD-800);
	led.green = map_LED(green,0,LED_PSEUDO_SCALE_MAX,0,LED_PWM_MAX_PERIOD_RELOAD);
	led.blue = map_LED(blue,0,LED_PSEUDO_SCALE_MAX,0,LED_PWM_MAX_PERIOD_RELOAD/*-599*/);
	led.white = map_LED(white,0,LED_PSEUDO_SCALE_MAX,0,LED_PWM_MAX_PERIOD_RELOAD);
}

/**
 * @brief  Maps a given value from one rage to another.
 * @param  x : value to map.
 * @param  in_min : Minimum value of x.
 * @param  in_max : Maximum value of x.
 * @param  out_min : Minimum value of x after mapping.
 * @param  out_max : Maximum value of x after mapping.
 * @retval uint32_t mapped value.
 */
uint32_t Led_driver::map_LED(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : ADC handle
 * @note   Adds the ADC readings into their respective sensor structures
 *					and calls the functions to calculate the moving averages for each sensor.
 * @retval None
 */
void Led_driver::adc_callback(ADC_HandleTypeDef *AdcHandle)
{
	temperature_sensor.raw_values[temperature_sensor.array_index%TELEM_MOVING_AVERAGE_BINS] = temperature_adc_reading;
	temperature_sensor.array_index = ++temperature_sensor.array_index % TELEM_MOVING_AVERAGE_BINS;
}

/**
 * @brief  Led driver constructor
 * @retval None
 */
Led_driver::Led_driver(void)
{
	// Initialise the driver state machine.
	prev_state = DRIVER_STATE_UNKNOWN;
	// All done.
	return;
}

