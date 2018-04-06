#include "ds18b20.h"
#include "ets_sys.h"
#include "gpio.h"
#include "osapi.h"

// --------------------------------------------------------------------------------

static bool ICACHE_FLASH_ATTR ds18b20_reset(ds18b20_t *sensor);
static inline void ICACHE_FLASH_ATTR ds18b20_write_bit(ds18b20_t *sensor, uint8_t bit);
static void ICACHE_FLASH_ATTR ds18b20_write_byte(ds18b20_t *sensor, uint8_t data, bool power_after_write);
static inline uint8_t ICACHE_FLASH_ATTR ds18b20_read_bit(ds18b20_t *sensor);
static uint8_t ICACHE_FLASH_ATTR ds18b20_read_byte(ds18b20_t *sensor);

// --------------------------------------------------------------------------------

#define READ_PIN(pin_no) (GPIO_INPUT_GET(pin_no) ? 1 : 0)
#define WRITE_HIGH(pin_no) GPIO_OUTPUT_SET(pin_no, 1)
#define WRITE_LOW(pin_no) GPIO_OUTPUT_SET(pin_no, 0)
#define MODE_OUTPUT(pin_no)
#define MODE_INPUT(pin_no) GPIO_DIS_OUTPUT(pin_no)

// --------------------------------------------------------------------------------

void ICACHE_FLASH_ATTR ds18b20_initialize(ds18b20_t *sensor, uint8_t gpio_pin)
{
	sensor->gpio_pin = gpio_pin;

	// Disable interrupts for the temperature sensor pin.
	gpio_pin_intr_state_set(GPIO_ID_PIN(sensor->gpio_pin), GPIO_PIN_INTR_DISABLE);

	// Enable pull-up sensor for the sensor pin.
	PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO0_U);

	// Leave the pin low initially.
	MODE_OUTPUT(sensor->gpio_pin);
	WRITE_LOW(sensor->gpio_pin);
}

void ICACHE_FLASH_ATTR ds18b20_request_temperature(ds18b20_t *sensor)
{
	// Reset the sensor bus. If no present signal is received, there is no sensor connected to the pin.
	if (!ds18b20_reset(sensor)) {
		return;
	}

	// Write a command to skip ROM (there is just the one sensor connected to the pin).
	ds18b20_write_byte(sensor, CMD_SKIP_ROM, false);

	// Write a command to initiate temperature conversion.
	ds18b20_write_byte(sensor, CMD_CONVERT_T, true);
}

void ICACHE_FLASH_ATTR ds18b20_read_temperature(ds18b20_t *sensor)
{
	// Reset the sensor bus. If no present signal is received, there is no sensor connected to the pin.
	if (!ds18b20_reset(sensor)) {
		return;
	}

	// Skip ROM, since there is just the one sensor connected.
	ds18b20_write_byte(sensor, CMD_SKIP_ROM, false);

	// Send a command to read the sensor's scratchpad.
	ds18b20_write_byte(sensor, CMD_READ_SCRATCHPAD, true);

	// Read 9 bytes from the device.
	int i;

	for (i = 0; i < SCRATCHPAD_SIZE; ++i) {
		sensor->scratchpad[i] = ds18b20_read_byte(sensor);
	}
}

float ICACHE_FLASH_ATTR ds18b20_get_temperature_c(ds18b20_t *sensor)
{
	// This function assumes 12-bit resolution temperature values.
	int16_t lsb = (int16_t)sensor->scratchpad[0]; // 8 bits from the least significant byte
	int16_t msb = (int16_t)(sensor->scratchpad[1] & 0xF); // 4 bits from the most significant byte
	int16_t raw = lsb | (msb << 8);

	float temp = raw * 0.0625f;

	// The most significant bit(s) of MSB contains the sign.
	if (sensor->scratchpad[1] & (1 << 7)) {
		temp *= -1;
	}

	return temp;
}

static bool ICACHE_FLASH_ATTR ds18b20_reset(ds18b20_t *sensor)
{
	// Let the pin float.
	MODE_INPUT(sensor->gpio_pin);
	os_delay_us(5);

	// Send a reset signal (pull to low for at least 480us).
	WRITE_LOW(sensor->gpio_pin);
	MODE_OUTPUT(sensor->gpio_pin);

	os_delay_us(480);

	// Let the sensor float again and start waiting for a signal from the sensor.
	// This should happen after 15 to 60us and last for 60-240us.
	MODE_INPUT(sensor->gpio_pin);
	
	os_delay_us(70);

	// If the pin is still floating, no device has responded to the reset signal.
	bool present = !READ_PIN(sensor->gpio_pin);

	os_delay_us(410);

	return present;
}

static inline void ICACHE_FLASH_ATTR ds18b20_write_bit(ds18b20_t *sensor, uint8_t bit)
{
	// Pull to low to begin a write slot.
	WRITE_LOW(sensor->gpio_pin);
	MODE_OUTPUT(sensor->gpio_pin);

	if (bit & 1) {
		// Write 1: pull the bus to low for a max of 15us, then let the pin float.
		os_delay_us(10);
		MODE_INPUT(sensor->gpio_pin);
		os_delay_us(55);
	}
	else {
		// Write 0: keep the bus at low for the duration of the time slot.
		os_delay_us(65);
		MODE_INPUT(sensor->gpio_pin);
		os_delay_us(5);
	}
}

static void ICACHE_FLASH_ATTR ds18b20_write_byte(ds18b20_t *sensor, uint8_t data, bool power_after_write)
{
	uint8_t mask;

	for (mask = 1; mask != 0; mask <<= 1) {
		ds18b20_write_bit(sensor, (mask & data) ? 1 : 0);
	}

	// Leave power on after a write.
	if (power_after_write) {
		MODE_OUTPUT(sensor->gpio_pin);
		WRITE_HIGH(sensor->gpio_pin);
	}
}

static inline uint8_t ICACHE_FLASH_ATTR ds18b20_read_bit(ds18b20_t *sensor)
{
	// Pull to low for at least 1us. This starts the read slot.
	MODE_OUTPUT(sensor->gpio_pin);
	WRITE_LOW(sensor->gpio_pin);

	os_delay_us(3);

	// Let the pin float, the sensor will pick this up and send the bit.
	MODE_INPUT(sensor->gpio_pin);

	os_delay_us(10);

	// In case of a 0 bit the sensor will pull the pin back to low.
	uint8_t bit = READ_PIN(sensor->gpio_pin);
	
	// Read time slot must be at least 60us long, hence the delay after the read.
	os_delay_us(53);

	return bit;
}

static uint8_t ICACHE_FLASH_ATTR ds18b20_read_byte(ds18b20_t *sensor)
{
	uint8_t mask, data = 0;

	for (mask = 1; mask != 0; mask <<= 1) {
		if (ds18b20_read_bit(sensor)) {
			data |= mask;
		}
	}

	return data;
}
