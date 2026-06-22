#include "MAX30205_driver.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len);
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data);

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len);
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data);

// --- Public Function Implementations ---

/**
 * @brief Initializes the MAX30205 sensor.
 */
void MAX30205_Init() {
	uint8_t config_value = 0b00000001;
	sensor_write_register(MAX30205_CONFIGURATION, config_value);
}

/**
 * @brief Start conversion of MAX30205 sensor.
 */
void MAX30205_Start_Conversion() {
	uint8_t config_value = 0b10000001;
	sensor_write_register(MAX30205_CONFIGURATION, config_value);
}

/**
 * @brief Reads the clinical temperature from the sensor and converts it to degrees Celsius.
 */
void MAX30205_Read_Temp(float *temperature, uint8_t *raw_to_mem, bool conversion_status) {
	uint8_t buffer[2];
	uint8_t is_conversion_finished = 0;
	int16_t raw_temp = 0;
	sensor_read_register(MAX30205_CONFIGURATION, &is_conversion_finished, 1);
	if (is_conversion_finished == 0b00000001) {
		sensor_read_register(MAX30205_TEMPERATURE, buffer, 2);
		raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
		raw_to_mem[0] = buffer[0];
		raw_to_mem[1] = buffer[1];
		conversion_status = false;

		// Conversion of the raw temperature value to degrees Celsius
		*temperature = (float)raw_temp * 0.00390625f;
	}
}

// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the IMU's register via I2C.
 *
 * This is a low-level function that encapsulates the I2C read operation.
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param data_len The number of bytes to read.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len) {
	if (HAL_I2C_Mem_Read(&hi2c3,
						 MAX30205_I2C_ADDR, // HAL manages the R/W bit
						 reg_addr,
						 I2C_MEMADD_SIZE_8BIT, // Register address on 1 byte
						 data,
						 data_len,
						 MAX30205_TIMEOUT) != HAL_OK) {
		return 0;
	}
	return 1;
}

/**
 * @brief Writes a single byte of data to a specific register on the IMU via I2C.
 *
 * @param reg_addr The address of the register to write to.
 * @param reg_data The byte of data to write.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data) {
	uint8_t tx_buffer[] = {reg_addr, reg_data};
	if (HAL_I2C_Master_Transmit(&hi2c3, MAX30205_I2C_ADDR, tx_buffer, sizeof(tx_buffer), MAX30205_TIMEOUT) != HAL_OK) {
		return 0; // Communication error
	}
	return 1; // Success
}
