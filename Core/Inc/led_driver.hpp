/*
 * led_driver.hpp
 *
 *  Created on: Oct 25, 2024
 *      Author: jmorritt
 */

#ifndef INC_LED_DRIVER_HPP_
#define INC_LED_DRIVER_HPP_

#include <stdint.h>

#include "driver_module.hpp"
#include "can_params.hpp"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_adc_ex.h"

#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "main.h"
#include "math.h"

#include <com.aeronavics.BoomStatus.h>
#include <uavcan.equipment.indication.LightsCommand.h>
#include <uavcan.equipment.actuator.ArrayCommand.h>
#include <uavcan.equipment.safety.ArmingStatus.h>
#include <ardupilot.indication.NotifyState.h>
#include <com.aeronavics.LoadcellInfo.h>



#define TELEM_MOVING_AVERAGE_BINS 			10
#define M_PI		3.14159265358979323846	/* pi */
#define M_2PI		6.283185307179586	/* pi */

// zero here comes from the flight controller. It is hard coded. Nice.
#define LIGHTS_COMMAND_STATUS_LIGHT_ID 0

#define LED_PSEUDO_SCALE_MAX 1000
#define LED_PWM_MAX_PERIOD_RELOAD 3599

#define OVERRIDE_TIMEOUT	1000 // ms - time since last overriding message was received before we go back to normally scheduled programming

#define FLASH_PATTERN_STARBOARD   0b10100010000010001010001000001000 //1010100000 0000010101 0000000000
#define FLASH_PATTERN_PORT        0b00001000101000100000100010100010 //0000000110 0110000000 0011001100

#define FLASH_PATTERN_STROBE	  0b00000000000000000000000000000101

#define FADE_MAX_BRIGHTNESS 250
#define FADE_PERIOD 4000	// ms

#define TEMP_ADC_MIN	0
#define TEMP_ADC_MAX	4095
#define ADC_VOLT_MIN	0.0
#define ADC_VOLT_MAX	3.3

#define KELVIN_OFFSET 273.15

#define TEMP_RESISTOR_T1 (KELVIN_OFFSET + 25.0)
#define TEMP_RESISTOR_R1 10000
#define TEMP_RESISTOR_B_CONSTANT 3380




typedef struct{
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t white;
}Colour_t;

enum OVERRIDE_PRIORITY{
	PRIORITY_NONE,
	PRIORITY_LOW,
	PRIORITY_MEDIUM,
	PRIORITY_HIGH,
	PRIORITY_HIGHEST,
};

enum OVERRIDE_TYPE{
	OVERRIDE_NONE,
	OVERRIDE_CAN,
};

typedef struct{
	Colour_t colour;
	bool is_active;
	uint8_t source_id;
	uint32_t last_override_time;
	OVERRIDE_PRIORITY priority;
	OVERRIDE_TYPE type;
}Override_t;

enum LED_MODE {
	LED_MODE_OFF, 		// Switches applicable LED's off.
	LED_MODE_RED_FADE,  // Fades the Red LED implying the can boom id has not been set up
    LED_MODE_FADE, 		// Fades the LED's in to the low brightness
	LED_MODE_STATUS,	// Displays flight controller status colours
    LED_MODE_NAV, 		// Uses a safe brightness set in LED_LOW_BRIGHTNESS setting
    LED_MODE_STROBE, 	// Full brightness white strobe pulse
	LED_MODE_OVERRIDE,	// CAN override
};

enum LED_CONTROL_MODE {
    LED_CONTROL_MODE_AUTO, 	// Lets the brightness be controlled via CAN Messages
    LED_CONTROL_MODE_MANUAL // Manually sets the brightness.
};

typedef struct{
	uint8_t array_index;		//current index in array for moving average
	uint32_t raw_values[TELEM_MOVING_AVERAGE_BINS];	//array of raw values used to calculate a moving average
}telemBase_t;

class Led_driver : public Driver_module
{
	public:
		void sync_update_unthrottled(void);
		void sync_update_100Hz(void);
		void sync_update_10Hz(void);
		void sync_update_1Hz(void);
		void handle_rx_can(const CanardRxTransfer * transfer, uint64_t data_type_signature, uint16_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

		void adc_callback(ADC_HandleTypeDef *AdcHandle);

		static Led_driver& get_driver(void);
		~Led_driver(void);

	private:

		Driver_state prev_state;

		uint16_t pwm_red;
		uint16_t pwm_green;
		uint16_t pwm_blue;
		uint16_t pwm_white;

		volatile uint32_t temperature_adc_reading;
		volatile telemBase_t temperature_sensor;
		float measured_temp;

		bool aircraft_armed;

		Colour_t status_colour;
		Override_t led_override;

		LED_MODE mode;

		uint8_t can_led_mode;

		uint8_t flash_pattern_index;

		uint32_t loopTimer;

		uint32_t last_can_received_time;
		bool isSystemRunning;
		bool isSystemArmed;

		Colour_t led;

		Led_driver(void);
		Led_driver(Led_driver const&);		// Poisoned.
		void operator =(Led_driver const&);	// Poisoned.

		void set_led_output(void);
		void update_LEDs(void);
		void setLED(uint16_t red, uint16_t green, uint16_t blue, uint16_t white);

		void transmit_telemetry(void);
		void calculate_temperature(void);

		float calculate_average(volatile uint32_t* array, uint8_t size);

		uint32_t map_LED(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
};

#endif /* INC_LED_DRIVER_HPP_ */
