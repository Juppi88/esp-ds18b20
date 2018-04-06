#ifndef __DS18B20_H
#define __DS18B20_H

#include "os_type.h"

#define SCRATCHPAD_SIZE 9

// --------------------------------------------------------------------------------

// OneWire commands for the temperature sensor.
enum ds_command_t {
	CMD_SEARCH_ROM = 0xF0,
	CMD_READ_ROM = 0x33,
	CMD_MATCH_ROM = 0x55,
	CMD_SKIP_ROM = 0xCC,
	CMD_ALARM_SEARCH = 0xEC,
	CMD_CONVERT_T = 0x44,
	CMD_WRITE_SCRATCHPAD = 0x4E,
	CMD_READ_SCRATCHPAD = 0xBE,
	CMD_COPY_SCRATCHPAD = 0x48,
	CMD_RECALL_E2 = 0xB8,
	CMD_READ_POWER_SUPPLY = 0xB4,
};

// --------------------------------------------------------------------------------

typedef struct {
	uint8_t gpio_pin;	// GPIO pin the sensor is connected to
	volatile uint8_t scratchpad[SCRATCHPAD_SIZE];	// Buffer for the sensor's scratchpad
} ds18b20_t;

// --------------------------------------------------------------------------------

// Initialize the sensor on the given GPIO pin.
void ICACHE_FLASH_ATTR ds18b20_initialize(ds18b20_t *sensor, uint8_t gpio_pin);

// Wake up the sensor for a value read.
void ICACHE_FLASH_ATTR ds18b20_request_temperature(ds18b20_t *sensor);

// Get the read temperature from the device and convert it to human readable units (C).
void ICACHE_FLASH_ATTR ds18b20_read_temperature(ds18b20_t *sensor);
float ICACHE_FLASH_ATTR ds18b20_get_temperature_c(ds18b20_t *sensor);

#endif
