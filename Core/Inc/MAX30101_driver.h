#ifndef __MAX30101_DRIVER_H__
#define __MAX30101_DRIVER_H__

/**
 * \file MAX30101_driver.h
 * \brief Driver library for the MAX30101 sensor.
 *
 * This header file provides the public interface for controlling the
 * MAX30101, including configuration and data reading functions.
 */

#include "stdint.h"
#include "main.h"
#include <stdint.h>

// I2C Addresses
#define MAX30101_WRITE_ADDRESS  0xAE //Scod: ho invertito AF e AE che li avevamo confusi
#define MAX30101_READ_ADDRESS   0xAF // 0xAE >> 1 and 0xAF >> 1 are the same address
#define MAX30101_TIMEOUT        100
// --- Register Configuration Values ---
// Interrupt Registers
#define INTERRUPT_STATUS_1      0x00
#define INTERRUPT_STATUS_2      0x01
#define INTERRUPT_ENABLE_1      0x02
#define INTERRUPT_ENABLE_2      0x03
// FIFO Registers
#define FIFO_WRITE_PTR          0x04
#define FIFO_OVF_COUNTER        0x05
#define FIFO_READ_PTR           0x06
#define FIFO_DATA_REG           0x07
// Configuration Registers
#define FIFO_CONFIGURATION      0x08
#define MODE_CONFIGURATION      0x09
#define SPO2_CONFIGURATION      0x0A
// LED Control
#define LED_PULSE_AMP           0x0C
#define MULTI_LED_CTRL          0x11

// --- Data Structures ---
// Width and Length of registers
#define NUMBER_OF_ACTIVE_LEDS  2
#define LED_PULSE_N_REG 4
#define MULTI_LED_N_REG 2
// This structure is used to store the Health data from MAX30101
typedef uint32_t HEALTH_data;


// --- Public Function Prototypes ---
// The main API for the MAX30101 driver.

/**
 * @brief Initializes the MAX30101 sensor and performs a PWR_RDY check.
 * @return 1 on success, 0 on failure.
 */
uint8_t MAX30101_Init(void);

/**
 * @brief Initializes the FIFO when changing mode.
 */
void MAX30101_Reset(void);

/**
 * @brief Configures the sensor.
 * @param fifo_conf Configures FIFO settings.
 * @param mode_conf Configures sensor mode (HR/SpO2/Both).
 * @param spo2_conf Configures SPO2 settings.
 */
void MAX30101_Mode_Config(uint8_t fifo_conf, uint8_t mode_conf, uint8_t spo2_conf);

/**
 * @brief Configures the sensor.
 * @param led_pa Configures LEDs pulse amplitude settings
 * @param multi_led Configures LEDs turn on-off stages 
 */
void MAX30101_LED_Config(uint8_t led_pa[LED_PULSE_N_REG], uint8_t multi_led[MULTI_LED_N_REG]);

/**
 * @brief Reads the raw and converted accelerometer data.
 * @param acc_data Pointer to an HEALTH_data struct where the results will be stored.
 * @param raw_data Pointer to an array used to store raw HEALTH data
 * @param read_ptr Pointer to last read position of FIFO
 */
uint8_t MAX30101_Read_Data(HEALTH_data acc_data[][32], uint8_t *raw_data, uint8_t *read_ptr); //Scod: read_ptr in teoria non necessario visto che il puntatore cambia da solo

#endif /* __MAX30101_DRIVER_H__ */
