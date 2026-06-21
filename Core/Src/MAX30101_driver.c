#include "MAX30101_driver.h"
#include "main.h"
#include "stm32u5xx_hal.h"
#include "string.h"
#include "stdio.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len);
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data, uint8_t mask);
// --- Public Function Implementations ---

/**
 * @brief Verify that the MAX30101 is ON
 *
 * @return 1 on success, 0 on failure.
 */
uint8_t MAX30101_Init(void) {
    uint8_t pwr_rdy_value = 0;
    uint32_t start_tick = HAL_GetTick();
    while(HAL_GetTick() - start_tick < 100){
        // Read the WHO_AM_I register and check if the returned value matches the expected one
        if (sensor_read_register(INTERRUPT_STATUS_1, &pwr_rdy_value, 1) && ((pwr_rdy_value & 1) == 0))
            return 1; // Success

        HAL_Delay(5);
    }
    return 0; // Failure
}

void MAX30101_Reset(void){
    sensor_write_register(FIFO_WRITE_PTR, 0, 0b00001111);
    sensor_write_register(FIFO_OVF_COUNTER, 0, 0b00001111);
    sensor_write_register(FIFO_READ_PTR, 0, 0b00001111);
}

/**
 * @brief Configures the sensor.
 * @param fifo_conf Configures FIFO settings.
 * @param mode_conf Configures sensor mode (HR/SpO2/Both).
 * @param spo2_conf Configures SPO2 settings.
 */
void MAX30101_Mode_Config(uint8_t fifo_conf, uint8_t mode_conf, uint8_t spo2_conf){
    // Combine ODR and FS into a single value for CTRL1_XL register
    sensor_write_register(INTERRUPT_ENABLE_1, 0, 0b11100000);
    sensor_write_register(INTERRUPT_ENABLE_2, 0, 0b00000010);
    sensor_write_register(FIFO_CONFIGURATION, fifo_conf, 0b11111111);
    sensor_write_register(MODE_CONFIGURATION, mode_conf, 0b11000111);
    sensor_write_register(SPO2_CONFIGURATION, spo2_conf, 0b01111111);

}

/**
 * @brief Configures the sensor.
 * @param led_pa Configures LEDs pulse amplitude settings
 * @param multi_led Configures LEDs turn on-off stages 
 */
void MAX30101_LED_Config(uint8_t led_pa[LED_PULSE_N_REG], uint8_t multi_led[MULTI_LED_N_REG]){
    for(int i = 0; i < LED_PULSE_N_REG; i++)
    {
        sensor_write_register((LED_PULSE_AMP + i), led_pa[i], 0b11111111);
    }
    for(int i = 0; i < MULTI_LED_N_REG; i++)
    {
        sensor_write_register((MULTI_LED_CTRL + i), multi_led[i], 0b01110111);
    }
}

/**
 * @brief Reads the raw and converted accelerometer data.
 * @param acc_data Pointer to an HEALTH_data struct where the results will be stored.
 * @param raw_data Pointer to an array used to store raw HEALTH data
 * @param read_ptr Pointer to last read position of FIFO
 */
uint8_t MAX30101_Read_Data(HEALTH_data acc_data[][32], uint8_t *raw_data, uint8_t *read_ptr){

    uint8_t     write_ptr, read_ptr_loc, ovf_counter;
    uint8_t     available_samples = 0;
    uint8_t     local_ptr = *read_ptr;

    sensor_read_register(FIFO_WRITE_PTR, &write_ptr, 1);
    sensor_read_register(FIFO_READ_PTR, &read_ptr_loc, 1);
    sensor_read_register(FIFO_OVF_COUNTER, &ovf_counter, 1);
    if (write_ptr >= local_ptr)
        available_samples = write_ptr - local_ptr;
    else 
        available_samples = 32 - local_ptr + write_ptr;
        
    for (uint8_t i = 0; i < available_samples; i++){
        sensor_read_register(FIFO_DATA_REG, raw_data, 3* NUMBER_OF_ACTIVE_LEDS);
        for (uint8_t j = 0; j < NUMBER_OF_ACTIVE_LEDS; j++){
            raw_data[0 + 3 * j] &= 0x3;
            // lo shift di 3 è per quando usiamo il formato a 15 bit, altrimenti è 0 (18 bit -> shift 0)
            acc_data[j][local_ptr] = ((raw_data[0 + 3 * j] << 16 | raw_data[1 + 3 * j] << 8 | raw_data[2 + 3 * j]) & 0x3FFFF) >> 0;
        }   

        local_ptr++;
        if (local_ptr == 32)
            local_ptr = 0;
    }
    *read_ptr = local_ptr;
    return available_samples;
}// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the IMU's register via I2C.
 *
 * This is a low-level function that encapsulates the I2C read operation.
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param data_len The number of bytes to read.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len) {
    if (HAL_I2C_Mem_Read(&hi2c3, 
                          MAX30101_READ_ADDRESS,    // 0xAE — HAL gestisce il bit R/W
                          reg_addr, 
                          I2C_MEMADD_SIZE_8BIT,      // indirizzo registro su 1 byte
                          data, 
                          data_len, 
                          MAX30101_TIMEOUT) != HAL_OK) {
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
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data, uint8_t mask) {
    uint8_t tx_buffer[] = {reg_addr, reg_data};
    uint8_t prev_data;
    sensor_read_register(reg_addr, &prev_data, 1);
    tx_buffer[1] = (reg_data & mask) | (prev_data & ~mask);
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30101_WRITE_ADDRESS, tx_buffer, sizeof(tx_buffer), MAX30101_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}
