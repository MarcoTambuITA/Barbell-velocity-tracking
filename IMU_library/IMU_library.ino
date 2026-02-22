/*
 * BMI270 Complete Driver - All-in-One Sketch
 * ===========================================
 * 
 * This is a COMPLETE, STANDALONE driver for BMI270 on Arduino Nano 33 BLE Rev2.
 * No external library required (except Wire.h which is built-in).
 * 
 * BEFORE UPLOADING:
 * You must create "BoschConfig.h" with the config array. See README.md.
 * 
 * WHAT THIS DOES:
 * - Initializes BMI270 at 400Hz sample rate
 * - Reads acceleration (g) and gyroscope (°/s)
 * - Prints timing stats to prove performance
 * 
 * Author: VBT Project
 */
#include <Wire.h>
// ============================================================================
// IMPORTANT: You must create this file! See README.md
// ============================================================================
#include "BoschConfig.h"
// ============================================================================
// BMI270 I2C Address and Register Definitions
// ============================================================================
#define BMI270_ADDR           0x68   // I2C address (default)
// Identification
#define REG_CHIP_ID           0x00   // Should return 0x24
// Data registers (12 bytes: AccX/Y/Z + GyrX/Y/Z, each 16-bit)
#define REG_ACC_X_LSB         0x0C
#define REG_ACC_X_MSB         0x0D
#define REG_ACC_Y_LSB         0x0E
#define REG_ACC_Y_MSB         0x0F
#define REG_ACC_Z_LSB         0x10
#define REG_ACC_Z_MSB         0x11
#define REG_GYR_X_LSB         0x12
#define REG_GYR_X_MSB         0x13
#define REG_GYR_Y_LSB         0x14
#define REG_GYR_Y_MSB         0x15
#define REG_GYR_Z_LSB         0x16
#define REG_GYR_Z_MSB         0x17
// Status
#define REG_INTERNAL_STATUS   0x21
// Configuration
#define REG_ACC_CONF          0x40   // Accel ODR and bandwidth
#define REG_ACC_RANGE         0x41   // Accel full-scale range
#define REG_GYR_CONF          0x42   // Gyro ODR and bandwidth
#define REG_GYR_RANGE         0x43   // Gyro full-scale range
// Initialization
#define REG_INIT_CTRL         0x59   // Config load control
#define REG_INIT_DATA         0x5E   // Config data burst write
// Power
#define REG_PWR_CONF          0x7C   // Advanced power config
#define REG_PWR_CTRL          0x7D   // Sensor enable (acc/gyr/aux/temp)
#define REG_CMD               0x7E   // Soft reset command
// ============================================================================
// Configuration Constants
// ============================================================================
// Accelerometer ODR (Output Data Rate): bits [3:0] of ACC_CONF
#define ACC_ODR_400HZ         0x0A
// Accelerometer Range: bits [1:0] of ACC_RANGE
#define ACC_RANGE_2G          0x00
#define ACC_RANGE_4G          0x01
#define ACC_RANGE_8G          0x02   // Good for barbell
#define ACC_RANGE_16G         0x03
// Gyroscope ODR: bits [3:0] of GYR_CONF
#define GYR_ODR_400HZ         0x0A
// Gyroscope Range: bits [2:0] of GYR_RANGE
#define GYR_RANGE_2000DPS     0x00
#define GYR_RANGE_1000DPS     0x01
#define GYR_RANGE_500DPS      0x02
#define GYR_RANGE_250DPS      0x03
#define GYR_RANGE_125DPS      0x04
// ============================================================================
// Scaling Factors (to convert raw 16-bit values to real units)
// ============================================================================
// For ±8g range: 1g = 32768/8 = 4096 LSB, so 1 LSB = 8/32768 g
const float ACC_SCALE_8G = 8.0f / 32768.0f;
// For ±2000°/s range: 2000 = 32768, so 1 LSB = 2000/32768 °/s
const float GYR_SCALE_2000DPS = 2000.0f / 32768.0f;
// ============================================================================
// Global Variables
// ============================================================================
// Timing
uint32_t lastPrintTime = 0;
uint32_t sampleCount = 0;
uint32_t minReadTime = 999999;
uint32_t maxReadTime = 0;
// ============================================================================
// Low-Level I2C Functions
// ============================================================================
/**
 * Write one byte to a register
 */
bool writeRegister(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(reg);
    Wire1.write(value);
    return (Wire1.endTransmission() == 0);
}
/**
 * Read one byte from a register
 */
bool readRegister(uint8_t reg, uint8_t *value) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    
    if (Wire1.requestFrom(BMI270_ADDR, (uint8_t)1) != 1) return false;
    *value = Wire1.read();
    return true;
}
/**
 * Read multiple bytes starting from a register (burst read)
 * This is THE FAST PATH for reading sensor data!
 */
uint8_t readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(startReg);
    if (Wire1.endTransmission(false) != 0) return 0;
    
    uint8_t received = Wire1.requestFrom(BMI270_ADDR, length);
    for (uint8_t i = 0; i < received; i++) {
        buffer[i] = Wire1.read();
    }
    return received;
}
// ============================================================================
// BMI270 Initialization
// ============================================================================
// Additional register for burst write index
#define REG_INIT_ADDR_0       0x5B   // Config load address LSB
#define REG_INIT_ADDR_1       0x5C   // Config load address MSB
/**
 * Load the config file to BMI270's internal RAM
 * This is required before the sensor will output data!
 * 
 * BMI270 Protocol: Must write word index to 0x5B-0x5C before each burst!
 */
bool loadConfigFile() {
    Serial.println("  Loading config file (8KB)...");
    
    // Prepare for burst write - disable loading first
    writeRegister(REG_INIT_CTRL, 0x00);
    delay(1);
    
    // Write config in smaller chunks with index addressing
    const uint16_t configSize = sizeof(bmi270_config_file);
    const uint8_t chunkSize = 32;  // Max bytes per I2C transaction
    
    for (uint16_t offset = 0; offset < configSize; offset += chunkSize) {
        // BMI270 CRITICAL: Write the word index (offset / 2) to address registers
        uint16_t wordIndex = offset / 2;
        writeRegister(REG_INIT_ADDR_0, (uint8_t)(wordIndex & 0xFF));
        writeRegister(REG_INIT_ADDR_1, (uint8_t)((wordIndex >> 8) & 0x0F));
        
        // Now write the data chunk
        Wire1.beginTransmission(BMI270_ADDR);
        Wire1.write(REG_INIT_DATA);
        
        for (uint8_t i = 0; i < chunkSize && (offset + i) < configSize; i++) {
            Wire1.write(bmi270_config_file[offset + i]);
        }
        
        if (Wire1.endTransmission() != 0) {
            Serial.print("  ERROR at offset ");
            Serial.println(offset);
            return false;
        }
        
        // Progress indicator every 2KB
        if (offset % 2048 == 0) Serial.print(".");
    }
    Serial.println(" Done!");
    
    // Signal config load complete
    writeRegister(REG_INIT_CTRL, 0x01);
    delay(150);  // Give BMI270 time to process config
    
    return true;
}
/**
 * Initialize BMI270 with 400Hz sampling
 */
bool initBMI270() {
    Serial.println("\n[BMI270] Starting initialization...");
    
    // Step 1: Verify chip ID
    uint8_t chipId;
    if (!readRegister(REG_CHIP_ID, &chipId)) {
        Serial.println("  ERROR: Cannot communicate with BMI270!");
        return false;
    }
    
    if (chipId != 0x24) {
        Serial.print("  ERROR: Wrong chip ID: 0x");
        Serial.println(chipId, HEX);
        return false;
    }
    Serial.println("  Chip ID verified: 0x24 ✓");
    
    // Step 2: Soft reset
    Serial.println("  Performing soft reset...");
    writeRegister(REG_CMD, 0xB6);
    delay(10);  // Wait for reset
    
    // Step 3: Disable advanced power save (required before config load)
    writeRegister(REG_PWR_CONF, 0x00);
    delayMicroseconds(500);
    
    // Step 4: Load config file (THE CRITICAL STEP!)
    if (!loadConfigFile()) {
        return false;
    }
    
    // Step 5: Wait and verify initialization
    delay(20);
    uint8_t status;
    readRegister(REG_INTERNAL_STATUS, &status);
    if ((status & 0x0F) != 0x01) {
        Serial.print("  WARNING: Internal status = 0x");
        Serial.println(status, HEX);
    } else {
        Serial.println("  Internal status: OK ✓");
    }
    
    // Step 6: Enable accelerometer + gyroscope + temperature
    writeRegister(REG_PWR_CTRL, 0x0E);  // Bits: temp=1, gyr=1, acc=1, aux=0
    delay(1);
    
    // Step 7: Configure accelerometer
    // ACC_CONF: [7:4]=BWP (0x02=normal), [3:0]=ODR (0x0A=400Hz)
    writeRegister(REG_ACC_CONF, 0x2A);  // 0x02 << 4 | 0x0A
    writeRegister(REG_ACC_RANGE, ACC_RANGE_8G);
    
    // Step 8: Configure gyroscope
    // GYR_CONF: [7:4]=BWP (0x02=normal), [3:0]=ODR (0x0A=400Hz)
    writeRegister(REG_GYR_CONF, 0x2A);  // 0x02 << 4 | 0x0A
    writeRegister(REG_GYR_RANGE, GYR_RANGE_2000DPS);
    
    Serial.println("\n[BMI270] Configuration:");
    Serial.println("  Accelerometer: ±8g @ 400Hz");
    Serial.println("  Gyroscope: ±2000°/s @ 400Hz");
    Serial.println("\n[BMI270] Ready! ✓");
    
    return true;
}
// ============================================================================
// Data Reading
// ============================================================================
/**
 * Read accelerometer and gyroscope data
 * Returns raw 16-bit values (convert using scale factors)
 */
bool readIMU(int16_t *accX, int16_t *accY, int16_t *accZ,
             int16_t *gyrX, int16_t *gyrY, int16_t *gyrZ) {
    
    uint8_t buffer[12];
    
    // Burst read all 12 bytes in one I2C transaction
    if (readRegisters(REG_ACC_X_LSB, buffer, 12) != 12) {
        return false;
    }
    
    // Parse little-endian 16-bit values
    *accX = (int16_t)((buffer[1] << 8) | buffer[0]);
    *accY = (int16_t)((buffer[3] << 8) | buffer[2]);
    *accZ = (int16_t)((buffer[5] << 8) | buffer[4]);
    *gyrX = (int16_t)((buffer[7] << 8) | buffer[6]);
    *gyrY = (int16_t)((buffer[9] << 8) | buffer[8]);
    *gyrZ = (int16_t)((buffer[11] << 8) | buffer[10]);
    
    return true;
}
// ============================================================================
// Arduino Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n==========================================");
    Serial.println("  BMI270 Complete - High-Speed Driver");
    Serial.println("  Arduino Nano 33 BLE Rev2");
    Serial.println("==========================================");
    
    // Initialize I2C (Wire1 = internal bus for onboard sensors)
    Wire1.begin();
    Wire1.setClock(400000);  // 400kHz Fast Mode
    Serial.println("\n[I2C] Wire1 initialized at 400kHz");
    
    // Initialize BMI270
    if (!initBMI270()) {
        Serial.println("\n[FATAL] BMI270 init failed! Halting.");
        while(1) delay(1000);
    }
    
    Serial.println("\n[INFO] Starting continuous sampling...");
    Serial.println("       Stats printed every 2 seconds.\n");
    
    lastPrintTime = millis();
}
void loop() {
    // Read sensor data
    uint32_t readStart = micros();
    
    int16_t ax, ay, az, gx, gy, gz;
    if (!readIMU(&ax, &ay, &az, &gx, &gy, &gz)) {
        Serial.println("[ERROR] Read failed!");
        return;
    }
    
    uint32_t readTime = micros() - readStart;
    
    // Track timing statistics
    if (readTime < minReadTime) minReadTime = readTime;
    if (readTime > maxReadTime) maxReadTime = readTime;
    sampleCount++;
    
    // Print stats every 2 seconds
    if (millis() - lastPrintTime >= 2000) {
        // Convert to real units
        float ax_g = ax * ACC_SCALE_8G;
        float ay_g = ay * ACC_SCALE_8G;
        float az_g = az * ACC_SCALE_8G;
        float gx_dps = gx * GYR_SCALE_2000DPS;
        float gy_dps = gy * GYR_SCALE_2000DPS;
        float gz_dps = gz * GYR_SCALE_2000DPS;
        
        float sampleRate = sampleCount / 2.0f;
        
        Serial.println("=============== STATS ===============");
        Serial.print("Sample Rate: ");
        Serial.print(sampleRate, 1);
        Serial.println(" Hz");
        
        Serial.print("I2C Read Time: ");
        Serial.print(minReadTime);
        Serial.print(" - ");
        Serial.print(maxReadTime);
        Serial.println(" µs");
        
        Serial.println("\nLatest Reading:");
        Serial.print("  Accel (g):   [");
        Serial.print(ax_g, 3); Serial.print(", ");
        Serial.print(ay_g, 3); Serial.print(", ");
        Serial.print(az_g, 3); Serial.println("]");
        
        Serial.print("  Gyro (°/s): [");
        Serial.print(gx_dps, 1); Serial.print(", ");
        Serial.print(gy_dps, 1); Serial.print(", ");
        Serial.print(gz_dps, 1); Serial.println("]");
        
        if (maxReadTime < 2500) {
            Serial.println("\n✓ Fast enough for 400Hz!");
        } else {
            Serial.println("\n⚠ Too slow for 400Hz target");
        }
        Serial.println("=====================================\n");
        
        // Reset stats
        lastPrintTime = millis();
        sampleCount = 0;
        minReadTime = 999999;
        maxReadTime = 0;
    }
    
    // Optional: Add delay to match 400Hz (2500µs period)
    // delayMicroseconds(2500 - readTime);
}
