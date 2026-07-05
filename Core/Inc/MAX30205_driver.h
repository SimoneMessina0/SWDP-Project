#ifndef __MAX30205_DRIVER_H__
#define __MAX30205_DRIVER_H__

/**
 * \file MAX30205_driver.h
 * \brief Driver library for the MAX30205 IMU sensor.
 *
 * This header file provides the public interface for controlling the
 * MAX30205, including configuration and data reading functions.
 */

#include "stdint.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// --- I2C Communication Settings ---
// The I2C slave address of the MAX30205 is 0x6B (7-bit format).
// This is the address to be used with the HAL library functions.
#define MAX30205_I2C_ADDR 0x90 
#define MAX30205_TIMEOUT   10 // Timeout in milliseconds for I2C operations

// --- Return Status Codes ---
#define MAX30205_STATUS_BUSY  0
#define MAX30205_STATUS_OK    1
#define MAX30205_STATUS_ERROR 2

// --- Register Addresses ---
// These are the addresses of the key registers used to configure and read the MAX30205.
#define MAX30205_TEMPERATURE   0x00 // Temperature data register (16-bit) 
#define MAX30205_CONFIGURATION 0x01 // Configuration register
#define MAX30205_THYST         0x02 // Hysteresis register for temperature threshold
#define MAX30205_TOS           0x03 // Over-temperature shutdown threshold register

// --- Public Function Prototypes ---
/**
 * @brief Initializes the MAX30205 sensor.
 */
void MAX30205_Init();

/**
 * @brief Start conversion of MAX30205 sensor.
 */
void MAX30205_Start_Conversion();

/**
 * @brief Extracts the clinical temperature from the sensor and converts it to degrees Celsius.
 * @param temperature Pointer to a float variable where the read temperature value will be stored in degrees Celsius.
 * @param raw_to_mem Pointer to 2-byte array to store raw temperature bytes.
 * @return 1 if conversion completed and temperature read, 0 otherwise.
 */
uint8_t MAX30205_Read_Temp(float *temperature, uint8_t *raw_to_mem);

#endif /* __MAX30205_DRIVER_H__ */
