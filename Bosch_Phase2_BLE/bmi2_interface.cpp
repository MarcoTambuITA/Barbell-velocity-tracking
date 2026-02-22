/**
 * @file bmi2_interface.cpp
 * @brief I2C Interface Wrapper Implementation for Bosch BMI270 API on Arduino
 * 
 * This file implements the "glue" functions that link the Bosch BMI270
 * Sensor API (which uses function pointers) to the Arduino Wire library.
 * 
 * Key functions:
 * - bmi2_i2c_read()  - Called by Bosch API to read registers
 * - bmi2_i2c_write() - Called by Bosch API to write registers
 * - bmi2_delay_us()  - Called by Bosch API for microsecond delays
 */
#include "bmi2_interface.h"
// ============================================================================
// I2C READ FUNCTION
// Called by Bosch API via function pointer to read sensor registers
// ============================================================================
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, 
                                     uint32_t len, void *intf_ptr) 
{
    // Cast interface pointer to our config structure
    struct bmi2_intf_config *cfg = (struct bmi2_intf_config *)intf_ptr;
    TwoWire *wire = cfg->wire;
    
    // Step 1: Send the register address we want to read from
    wire->beginTransmission(cfg->i2c_addr);
    wire->write(reg_addr);
    
    // End transmission with repeated start (false = don't release bus)
    // This is crucial for proper I2C read sequence
    if (wire->endTransmission(false) != 0) {
        return BMI2_E_COM_FAIL;
    }
    
    // Step 2: Request 'len' bytes from the sensor
    uint32_t bytes_received = wire->requestFrom(cfg->i2c_addr, (size_t)len);
    
    if (bytes_received != len) {
        return BMI2_E_COM_FAIL;
    }
    
    // Step 3: Read all bytes into the buffer
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = wire->read();
    }
    
    return BMI2_INTF_RET_SUCCESS;
}
// ============================================================================
// I2C WRITE FUNCTION
// Called by Bosch API via function pointer to write sensor registers
// ============================================================================
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, 
                                      uint32_t len, void *intf_ptr) 
{
    // Cast interface pointer to our config structure
    struct bmi2_intf_config *cfg = (struct bmi2_intf_config *)intf_ptr;
    TwoWire *wire = cfg->wire;
    
    // Begin I2C transmission to the sensor
    wire->beginTransmission(cfg->i2c_addr);
    
    // First byte is the register address
    wire->write(reg_addr);
    
    // Write all data bytes
    for (uint32_t i = 0; i < len; i++) {
        wire->write(reg_data[i]);
    }
    
    // End transmission and check for errors
    // Return 0 means success
    uint8_t result = wire->endTransmission();
    
    return (result == 0) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}
// ============================================================================
// DELAY FUNCTION (microseconds)
// Called by Bosch API for timing-critical operations
// ============================================================================
void bmi2_delay_us(uint32_t period, void *intf_ptr) 
{
    (void)intf_ptr;  // Unused parameter
    
    // For very short delays, use delayMicroseconds
    // For longer delays (>16ms), use delay() to allow other tasks
    if (period > 16000) {
        delay(period / 1000);
        delayMicroseconds(period % 1000);
    } else {
        delayMicroseconds(period);
    }
}
// ============================================================================
// INTERFACE INITIALIZATION
// Sets up the bmi2_dev structure with our platform functions
// ============================================================================
int8_t bmi2_interface_init(struct bmi2_dev *dev, struct bmi2_intf_config *intf_cfg) 
{
    // Validate pointers
    if (dev == NULL || intf_cfg == NULL || intf_cfg->wire == NULL) {
        return BMI2_E_NULL_PTR;
    }
    
    // Initialize I2C bus
    intf_cfg->wire->begin();
    
    // Set I2C clock to 400kHz (Fast Mode)
    // The nRF52840 on Nano 33 BLE supports up to 400kHz standard
    // You can try 1000000 (1MHz Fast Mode Plus) but results may vary
    intf_cfg->wire->setClock(1000000);
    
    // Configure the device structure for the Bosch API
    dev->intf = BMI2_I2C_INTF;           // We're using I2C interface
    dev->intf_ptr = intf_cfg;             // Pointer to our config struct
    dev->read = bmi2_i2c_read;            // Our read function
    dev->write = bmi2_i2c_write;          // Our write function
    dev->delay_us = bmi2_delay_us;        // Our delay function
    
    // Maximum bytes per I2C transaction
    // The Wire library buffer is typically 32 bytes
    dev->read_write_len = 32;
    
    return BMI2_OK;
}
