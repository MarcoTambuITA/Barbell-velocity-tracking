#include "bmi2.h"

#ifndef BMI2_CALIBRATION_H
#define BMI2_CALIBRATION_H


/**
 * @brief Perform Fast Offset Compensation (FOC) for accelerometer and gyroscope
 * 
 * Place the sensor FLAT and STATIONARY on a level surface before calling.
 * Z-axis should be pointing UP (away from table).
 * 
 * The calibration offsets are stored in sensor registers and applied
 * automatically to all subsequent readings.
 */
int8_t performFOC(struct bmi2_dev *dev) {
    int8_t rslt;
    
    // ---- Step 1: Perform Gyroscope FOC ----
    // Gyro must be stationary (no rotation)
    Serial.print("Gyro FOC... ");
    rslt = bmi2_perform_gyro_foc(dev);
    if (rslt != BMI2_OK) {
        Serial.print("FAILED: "); Serial.println(rslt);
        return rslt;
    }
    Serial.println("OK");
    
    // ---- Step 2: Perform Accelerometer FOC ----
    // Define expected gravity vector (Z-up = +1g on Z-axis)
    struct bmi2_accel_foc_g_value g_value;
    g_value.x = 0;      // No gravity on X
    g_value.y = 0;      // No gravity on Y
    g_value.z = 1;      // +1g on Z (sensor flat, Z pointing up)
    g_value.sign = 0;   // Positive direction
    
    Serial.print("Accel FOC... ");
    rslt = bmi2_perform_accel_foc(&g_value, dev);
    if (rslt != BMI2_OK) {
        Serial.print("FAILED: "); Serial.println(rslt);
        return rslt;
    }
    Serial.println("OK");
    
    Serial.println("FOC Complete! Offsets stored in sensor registers.");
    return BMI2_OK;
}

#endif
