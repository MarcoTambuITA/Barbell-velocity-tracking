/**
 * @file bmi2_interface.h
 * @brief I2C Interface Wrapper for Bosch BMI270 API on Arduino
 * 
 * This header provides the "glue" functions that link the Bosch BMI270
 * Sensor API (which uses function pointers) to the Arduino Wire library.
 * 
 * Designed for Arduino Nano 33 BLE Rev2 with internal I2C bus (Wire1).
 */
#ifndef BMI2_INTERFACE_H
#define BMI2_INTERFACE_H
#include <Arduino.h>
#include <Wire.h>
// Include Bosch API definitions using extern "C" for C linkage
extern "C" {
    #include "bmi2_defs.h"
}
// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
/**
 * @brief BMI270 I2C address on Nano 33 BLE Rev2 internal bus
 * 
 * The BMI270 supports two I2C addresses based on the SDO pin:
 * - 0x68 (SDO = GND) - Default on Arduino Nano 33 BLE
 * - 0x69 (SDO = VCC)
 */
#define BMI270_I2C_ADDR  0x68
// ============================================================================
// INTERFACE CONTEXT STRUCTURE
// ============================================================================
/**
 * @brief Interface configuration passed to Bosch API via intf_ptr
 * 
 * This structure holds the platform-specific details needed by our
 * read/write functions to communicate with the sensor.
 */
struct bmi2_intf_config {
    TwoWire* wire;       // Pointer to Wire instance (Wire or Wire1)
    uint8_t i2c_addr;    // I2C address of the BMI270
};
// ============================================================================
// FUNCTION DECLARATIONS - Bosch API Function Pointer Compatible
// ============================================================================
/**
 * @brief I2C read function for Bosch API
 * 
 * Matches the signature: bmi2_read_fptr_t
 * 
 * @param reg_addr  Register address to read from
 * @param reg_data  Buffer to store read data
 * @param len       Number of bytes to read
 * @param intf_ptr  Pointer to bmi2_intf_config structure
 * @return BMI2_INTF_RET_SUCCESS on success, error code on failure
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, 
                                     uint32_t len, void *intf_ptr);
/**
 * @brief I2C write function for Bosch API
 * 
 * Matches the signature: bmi2_write_fptr_t
 * 
 * @param reg_addr  Register address to write to
 * @param reg_data  Data buffer to write
 * @param len       Number of bytes to write
 * @param intf_ptr  Pointer to bmi2_intf_config structure
 * @return BMI2_INTF_RET_SUCCESS on success, error code on failure
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, 
                                      uint32_t len, void *intf_ptr);
/**
 * @brief Delay function for Bosch API (microseconds)
 * 
 * Matches the signature: bmi2_delay_fptr_t
 * 
 * @param period    Delay period in microseconds
 * @param intf_ptr  Interface pointer (unused)
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr);
// ============================================================================
// HIGH-LEVEL INITIALIZATION
// ============================================================================
/**
 * @brief Initialize the BMI2 interface
 * 
 * Sets up the bmi2_dev structure with our platform-specific I2C functions
 * and configures the I2C bus for high-speed operation.
 * 
 * @param dev       Pointer to bmi2_dev structure to initialize
 * @param intf_cfg  Pointer to interface configuration (caller must maintain)
 * @return BMI2_OK on success, error code on failure
 */
int8_t bmi2_interface_init(struct bmi2_dev *dev, struct bmi2_intf_config *intf_cfg);
#endif // BMI2_INTERFACE_H
