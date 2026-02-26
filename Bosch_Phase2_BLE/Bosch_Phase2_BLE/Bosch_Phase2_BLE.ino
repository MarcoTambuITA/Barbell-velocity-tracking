#include "bmi2_calibration.h"
#include <Wire.h>
#include "bmi2_interface.h"
#include "MadgwickAHRS.h"
#include "ble_vbt_service.h"
// Include Bosch API with C linkage
extern "C" {
    #include "bmi270.h"
    #include "bmi2.h"
    
}

struct bmi2_dev bmi2;
// Our interface configuration
struct bmi2_intf_config intf_cfg;
// Sensor data structure for readings
struct bmi2_sens_data sensor_data;

// Sensor settings - adjust based on your VBT requirements
#define ACCEL_RANGE_G       8       // ±8g range (good for barbell movements)
#define GYRO_RANGE_DPS      2000    // ±2000°/s range
// Unit conversion constants
// Accelerometer: raw * (GRAVITY * RANGE / 32768)
// Gyroscope: raw * (RANGE / 32768)
#define GRAVITY_MS2         9.80665f
#define ACCEL_SCALE         (GRAVITY_MS2 * ACCEL_RANGE_G / 32768.0f)   // → m/s²
#define GYRO_SCALE          (GYRO_RANGE_DPS / 32768.0f)                 // → °/s
#define GYRO_SCALE_RAD      (GYRO_RANGE_DPS * 0.01745329f / 32768.0f)  // → rad/s
// Timing variables
volatile uint32_t sample_count = 0;
uint32_t last_report_time = 0;

// Current vertical velocity — updated by IMU pipeline, read by BLE
float currentVz = 0.0f;

// ════════════════════════════════════════════════════════
// REP DETECTION FSM
// ════════════════════════════════════════════════════════
enum RepState { REP_IDLE, REP_ECCENTRIC, REP_CONCENTRIC };

// FSM Thresholds (tune based on testing)
const float ECCENTRIC_THRESH  = 0.2f;   // m/s — vel_z below -this enters eccentric
const float CONCENTRIC_THRESH = 0.18f;   // m/s — vel_z above +this enters concentric (from IDLE)
const float LOCKOUT_THRESH    = 0.1f;   // m/s — vel_z below this ends concentric (lockout)
const float MIN_ROM_M         = 0.15f;   // meters — minimum displacement for a valid rep 15cm
const uint32_t SET_TIMEOUT_MS = 5000;    // 7s of IDLE+ZUPT = set complete


void setup() {
    // Initialize serial at high baud rate for fast debugging
    Serial.begin(2000000);
    
    // Wait for Serial connection (with timeout for standalone operation)
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 3000);
    
    
    // ---- Step 1: Configure Interface ----
    
    intf_cfg.wire = &Wire1;              // Internal I2C bus on Nano 33 BLE
    intf_cfg.i2c_addr = BMI270_I2C_ADDR; // 0x68
    
    int8_t rslt = bmi2_interface_init(&bmi2, &intf_cfg);
    if (rslt != BMI2_OK) {
        Serial.print("FAILED (");
        Serial.print(rslt);
        Serial.println(")");
        haltWithError("Interface initialization failed");
    }

    
    // ---- Step 2: Initialize BMI270 ----
    
    rslt = bmi270_init(&bmi2);
    if (rslt != BMI2_OK) {
        Serial.print("FAILED (");
        Serial.print(rslt);
        Serial.println(")");
        printError(rslt);
        haltWithError("BMI270 initialization failed");
    }
    
    // Verify chip ID
    Serial.print("   Chip ID: 0x");
    Serial.print(bmi2.chip_id, HEX);
    if (bmi2.chip_id == BMI270_CHIP_ID) {
    } else {
        Serial.println(" (WARNING: Unexpected chip ID!)");
    }
    
    // ---- Step 3: Configure Sensors ----
    Serial.print("3. Configuring sensors for high-speed... ");
    
    rslt = configureSensors();
    if (rslt != BMI2_OK) {
        Serial.print("FAILED (");
        Serial.print(rslt);
        Serial.println(")");
        haltWithError("Sensor configuration failed");
    }
    
    // ---- Step 4: Enable Sensors ----   
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &bmi2);
    if (rslt != BMI2_OK) {
        Serial.print("FAILED (");
        Serial.print(rslt);
        Serial.println(")");
        haltWithError("Sensor enable failed");
    }
    
    // ---- Print Configuration Summary ----
    Serial.print("  Accel Range: ±"); Serial.print(ACCEL_RANGE_G); Serial.println("g");
    Serial.print("  Gyro Range:  ±"); Serial.print(GYRO_RANGE_DPS); Serial.println("°/s");
    
Serial.println("Place sensor FLAT and STILL. Press any key to calibrate...");
    while (!Serial.available());
    Serial.read();  // Clear the input
    
    performFOC(&bmi2);

    // Allow sensors to stabilize
    delay(100);
    
    // ---- Step 5: Initialize BLE ----
    if (!initBLE()) {
        haltWithError("BLE initialization failed");
    }

    last_report_time = millis();
}

// Configure for maximum performance at 400Hz
int8_t configureSensors() {
    int8_t rslt;
    struct bmi2_sens_config sens_cfg[2];
    
    // ---- Accelerometer Configuration ----
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;         // 400 Hz output data rate
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_8G;        // ±8g range
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;       // Normal bandwidth, 4 samples avg
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // PERFORMANCE mode (not power-optimized)
    
    // ---- Gyroscope Configuration ----
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;         // 400 Hz output data rate
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;      // ±2000°/s range
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;       // Normal bandwidth
    sens_cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;  // Low-noise mode
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // PERFORMANCE mode
    
    // Apply configuration
    rslt = bmi2_set_sensor_config(sens_cfg, 2, &bmi2);
    
    return rslt;
}

void loop() {
    uint32_t nowMs = millis();

    // ── BLE: Service incoming events & connection checks (every 50ms) ──
    static uint32_t lastBlePollMs = 0;
    static BLEDevice central;
    
    if (nowMs - lastBlePollMs >= 50) {
        lastBlePollMs = nowMs;
        BLE.poll();
        central = BLE.central();
    }

    // ── BLE: Data update (every 100ms if connected) ──
    static uint32_t lastBleUpdateMs = 0;
    static bool wasBleConnected = false;
    bool isBleConnected = (central && central.connected());

    if (isBleConnected) {
        if (nowMs - lastBleUpdateMs >= 100) {
            lastBleUpdateMs = nowMs;
            velocityChar.writeValue(currentVz);
        }
    } else {
        // Not connected — advertise only if we just disconnected
        if (wasBleConnected) {
            BLE.advertise();
        }
    }
    wasBleConnected = isBleConnected;

    int8_t rslt;
    
    // Only proceed if new data is available
    uint8_t status;
    bmi2_get_regs(BMI2_STATUS_ADDR, &status, 1, &bmi2);
    
    if (!(status & 0x80)) {  // Bit 7 = ACC data ready
        return;  // No new data, skip this iteration
    }


    // Read accelerometer and gyroscope data in one call
    // This reads from data registers in a burst for efficiency
    rslt = bmi2_get_sensor_data(&sensor_data, &bmi2);
    
    if (rslt == BMI2_OK) {
        sample_count++;
        

        uint32_t loopStartMicros = micros();
        // ════════════════════════════════════════════════════════
        // TIMING CODE GOES HERE (at the start of the if block)
        // ════════════════════════════════════════════════════════
        static uint32_t lastMicros = 0;
        uint32_t nowMicros = micros();
        float dt = (nowMicros - lastMicros) / 1000000.0f;
        lastMicros = nowMicros;
        
        if (dt <= 0.0f || dt > 0.05f) dt = 0.0025f;   // only clamp truly insane values (>50 ms)
        // ════════════════════════════════════════════════════════


        // CONVERT RAW DATA TO REAL UNITS      
        // Accelerometer: int16 raw → m/s²
        float acc_x = sensor_data.acc.x * ACCEL_SCALE;
        float acc_y = sensor_data.acc.y * ACCEL_SCALE;
        float acc_z = sensor_data.acc.z * ACCEL_SCALE;
        
        // Gyroscope: int16 raw → °/s
        float gyr_x_rad = sensor_data.gyr.x * GYRO_SCALE_RAD;
        float gyr_y_rad = sensor_data.gyr.y * GYRO_SCALE_RAD;
        float gyr_z_rad = sensor_data.gyr.z * GYRO_SCALE_RAD;
        
    MadgwickAHRSupdateIMU(gyr_x_rad, gyr_y_rad, gyr_z_rad, acc_x, acc_y, acc_z, dt);


// After calling MadgwickAHRSupdateIMU():
// q0, q1, q2, q3 are now updated - use them to rotate gravity out
// Example: Rotate gravity vector [0, 0, 9.81] from Earth frame to sensor frame
float gx_sensor = 2.0f * (q1*q3 - q0*q2) * GRAVITY_MS2;
float gy_sensor = 2.0f * (q0*q1 + q2*q3) * GRAVITY_MS2;
float gz_sensor = (q0*q0 - q1*q1 - q2*q2 + q3*q3) * GRAVITY_MS2;
// Subtract gravity to get linear acceleration
float linear_acc_x = acc_x - gx_sensor;
float linear_acc_y = acc_y - gy_sensor;
float linear_acc_z = acc_z - gz_sensor;

// ════════════════════════════════════════════════════════
// WORLD FRAME PROJECTION: Sensor Frame → Earth Frame (Full 3D)
// ════════════════════════════════════════════════════════
// Rotation matrix from quaternion (sensor → Earth):
// Row 1 (Earth X): [1-2(q2²+q3²),  2(q1q2-q0q3),  2(q1q3+q0q2)]
// Row 2 (Earth Y): [2(q1q2+q0q3),  1-2(q1²+q3²),  2(q2q3-q0q1)]
// Row 3 (Earth Z): [2(q1q3-q0q2),  2(q0q1+q2q3),  1-2(q1²+q2²)]

float acc_earth_x = linear_acc_x * (1.0f - 2.0f*(q2*q2 + q3*q3))
                  + linear_acc_y * 2.0f*(q1*q2 - q0*q3)
                  + linear_acc_z * 2.0f*(q1*q3 + q0*q2);

float acc_earth_y = linear_acc_x * 2.0f*(q1*q2 + q0*q3)
                  + linear_acc_y * (1.0f - 2.0f*(q1*q1 + q3*q3))
                  + linear_acc_z * 2.0f*(q2*q3 - q0*q1);

float acc_earth_z = linear_acc_x * 2.0f*(q1*q3 - q0*q2)
                  + linear_acc_y * 2.0f*(q0*q1 + q2*q3)
                  + linear_acc_z * (1.0f - 2.0f*(q1*q1 + q2*q2));

// acc_earth_z = True Vertical (positive = UP)
// acc_earth_x/y = Horizontal plane (for bar path / lateral deviation)

        // ════════════════════════════════════════════════════════
        // ACCELERATION DEADZONE (drift killer)
        // ════════════════════════════════════════════════════════
        // Forces below this threshold are sensor noise / tilt error,
        // not real human movement. Zero them to prevent ghost drift.
        const float ACC_DEADZONE = 0.05f;  // m/s²
        if (fabsf(acc_earth_x) < ACC_DEADZONE) acc_earth_x = 0.0f;
        if (fabsf(acc_earth_y) < ACC_DEADZONE) acc_earth_y = 0.0f;
        if (fabsf(acc_earth_z) < ACC_DEADZONE) acc_earth_z = 0.0f;

        // ================================================================
        //Your VBT Code goes HERE
        // VELOCITY INTEGRATION: V = V + (a * dt) 
    // ================================================================
        static float vel_x = 0.0f;
        static float vel_y = 0.0f;
        static float vel_z = 0.0f;
        
        vel_x += acc_earth_x * dt;
        vel_y += acc_earth_y * dt;
        vel_z += acc_earth_z * dt;
        currentVz = vel_z; // Update global for BLE tracking

        // ================================================================
        // ZUPT: Zero Velocity Update (drift correction)
        // ================================================================
        // When the bar is stationary, reset velocity to zero.
        // Detection: low acceleration + low rotation for minimum duration.
        
        // Thresholds (tune these based on testing)
        const float ACCEL_STILLNESS_THRESH = 0.4f;   // m/s² - max accel magnitude when "still"
        const float GYRO_STILLNESS_THRESH = 0.15f;   // rad/s - max gyro magnitude when "still"
        const uint32_t ZUPT_MIN_DURATION_MS = 100;   // Must be still for this long
        
        // Calculate magnitudes
        static RepState rep_state = REP_IDLE;
        // For stillness detection, use raw sensor acceleration (bypasses orientation drift)
        float raw_acc_mag = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
        float gyro_mag = sqrtf(gyr_x_rad*gyr_x_rad + gyr_y_rad*gyr_y_rad + gyr_z_rad*gyr_z_rad);
        
        // Track how long we've been "still"
        static uint32_t stillness_start_ms = 0;
        static bool was_still = false;
        static bool zupt_active = false;
        
        // Still if raw acceleration is near 1.0G (9.81 m/s^2) and rotation is minimal
        bool is_still_now = (fabsf(raw_acc_mag - GRAVITY_MS2) < 0.5f) && (gyro_mag < 0.2f);
        
        if (is_still_now) {
            if (!was_still) {
                // Just became still - start the timer
                stillness_start_ms = millis();
            } else if (!zupt_active && (millis() - stillness_start_ms >= ZUPT_MIN_DURATION_MS)) {
                // Been still long enough - apply ZUPT!
                vel_x = 0.0f;
                vel_x = 0.0f;
                vel_y = 0.0f;
                vel_z = 0.0f;
                zupt_active = true;
                rep_state = REP_IDLE; // FSM Recovery: force IDLE state when stationary
            }
            // If zupt_active, keep velocity at zero while still
            if (zupt_active) {
                vel_x = 0.0f;
                vel_y = 0.0f;
                vel_z = 0.0f;
                rep_state = REP_IDLE; // Keep FSM in IDLE while still
            }
        } else {
            // Moving - deactivate ZUPT
            zupt_active = false;
        }
        was_still = is_still_now;
        
        // ────────────────────────────────────────────────────────
        // GENTLE VELOCITY DECAY (anti-drift during low activity)
        // ────────────────────────────────────────────────────────
        // When acceleration is moderate (not a real lift, but not still enough for ZUPT),
        // apply a very gentle decay to prevent runaway drift.
        // 0.998 per sample at 400Hz ≈ 56% reduction per second
        bool decay_active = (fabsf(raw_acc_mag - GRAVITY_MS2) < 1.0f && !zupt_active);
        if (decay_active) {
            vel_x *= 0.99f;      // Aggressive horizontal decay (drift killer)
            vel_y *= 0.99f;      // Aggressive horizontal decay (drift killer)
            
            vel_z *= 0.998f;     // Light decay always — still preserves lift dynamics
        }
        // ════════════════════════════════════════════════════════
        // REP DETECTION FSM
        // ════════════════════════════════════════════════════════
      
        static float    conc_vel_sum      = 0.0f;   // for Mean Concentric Velocity
        static float    conc_peak_vel     = 0.0f;   // for Peak Concentric Velocity
        static float    conc_prop_acc_sum = 0.0f;   // for Mean Propulsive Z Acceleration (m/s^2)
        static float    conc_peak_acc     = 0.0f;   // for Peak Z Acceleration (m/s^2)
        static float    conc_displacement = 0.0f;   // for Range of Motion (meters)
        static uint32_t conc_samples      = 0;      // sample count during concentric
        static uint32_t prop_samples      = 0;      // sample count during propulsive phase
        static uint32_t rep_count         = 0;      // total valid reps detected
        static float    rep_tut           = 0.0f;   // Time Under Tension for current rep (s)
        static float    set_tut           = 0.0f;   // Cumulative TUT for the entire set (s)
        static uint32_t idle_start_ms     = 0;       // When IDLE+ZUPT phase began
        static bool     idle_timing       = false;   // Are we tracking IDLE duration?
        static bool     set_active        = false;   // Has at least 1 rep been completed?

        switch (rep_state) {
            case REP_IDLE:
                if (vel_z < -ECCENTRIC_THRESH) {
                    // Bar moving down → eccentric phase (squat, bench, etc.)
                    rep_state = REP_ECCENTRIC;
                    rep_tut = 0.0f;    // Start TUT clock for new rep
                    idle_timing = false; // Cancel any set-timeout timer
                } else if (vel_z > CONCENTRIC_THRESH) {
                    // Bar moving up from rest → concentric-only lift (deadlift, clean)
                    rep_state = REP_CONCENTRIC;
                    conc_vel_sum = 0.0f;
                    conc_peak_vel = 0.0f;
                    conc_prop_acc_sum = 0.0f;
                    conc_peak_acc = 0.0f;
                    conc_displacement = 0.0f;
                    conc_samples = 0;
                    prop_samples = 0;
                    rep_tut = 0.0f;    // Start TUT clock for concentric-only rep
                    idle_timing = false; // Cancel any set-timeout timer
                }
                break;

            case REP_ECCENTRIC:
                rep_tut += dt;  // Accumulate TUT during eccentric phase
                if (vel_z > 0.0f) {     // true zero-crossing: bar just reversed direction
                    // Zero-crossing = bottom of rep → transition to concentric
                    rep_state = REP_CONCENTRIC;
                    conc_vel_sum = 0.0f;
                    conc_peak_vel = 0.0f;
                    conc_prop_acc_sum = 0.0f;
                    conc_peak_acc = 0.0f;
                    conc_displacement = 0.0f;
                    conc_samples = 0;
                    prop_samples = 0;
                }
                break;

            case REP_CONCENTRIC:
                // Accumulate metrics every sample
                rep_tut += dt;   // Accumulate TUT during concentric phase
                conc_vel_sum += vel_z;
                if (acc_earth_z > 0.0f) {
                    conc_prop_acc_sum += acc_earth_z;
                    prop_samples++;
                }
                conc_samples++;
                conc_displacement += vel_z * dt;
                if (vel_z > conc_peak_vel) conc_peak_vel = vel_z;
                if (acc_earth_z > conc_peak_acc) conc_peak_acc = acc_earth_z;

                // Check for rep completion: velocity dropping back toward zero (lockout)
                if (vel_z < LOCKOUT_THRESH) {
                    if (conc_displacement > MIN_ROM_M && conc_samples > 0) {
                        // ── Valid rep! Report metrics ──
                        rep_count++;
                        set_active = true;  // At least 1 valid rep → set is active
                        float MCV = conc_vel_sum / (float)conc_samples;
                        float avgZAccel = prop_samples > 0 ? (conc_prop_acc_sum / (float)prop_samples) : 0.0f;

                        // Send to BLE (isSetComplete = false, because this is just a rep finishing)
                        RepData stats = {MCV, conc_peak_vel, rep_tut, conc_displacement, avgZAccel, conc_peak_acc, rep_count, false};
                        updateRepStats(stats);

                        // Consolidated Serial Report (faster than 15 separate calls)
                        Serial.print(">>> REP #"); Serial.print(rep_count);
                        Serial.print(" | MCV:"); Serial.print(MCV, 3);
                        Serial.print(" PCV:"); Serial.print(conc_peak_vel, 3);
                        Serial.print(" ROM:"); Serial.print(conc_displacement * 100.0f, 1);
                        Serial.print(" TUT:"); Serial.print(rep_tut, 2);
                        Serial.print(" SetTUT:"); Serial.println(set_tut + rep_tut, 1);
                        
                        set_tut += rep_tut;
                    }
                    // Reset to IDLE (valid or not — jitter is silently discarded)
                    rep_state = REP_IDLE;
                }
                break;
        }

        // ════════════════════════════════════════════════════════
        // SET COMPLETION TIMEOUT
        // ════════════════════════════════════════════════════════
        // If IDLE + ZUPT active + set has reps → start/continue timeout.
        // After SET_TIMEOUT_MS of continuous stillness, declare set complete.
        if (rep_state == REP_IDLE && zupt_active && set_active) {
            if (!idle_timing) {
                idle_start_ms = millis();
                idle_timing = true;
            } else if (millis() - idle_start_ms >= SET_TIMEOUT_MS) {
                // ── Set Complete! Print summary ──
                Serial.println();
                Serial.println("══════════════ SET COMPLETE ══════════════");
                Serial.print("  Total Reps: ");
                Serial.print(rep_count);
                Serial.print(" | Total TUT: ");
                Serial.print(set_tut, 2);
                Serial.println(" s");
                Serial.println("══════════════════════════════════════════");
                Serial.println();
                
                // Send "Set Complete" flag via BLE
                // Zero out the velocity/accel metrics; the important parts are rep_count and isSetComplete=true
                RepData finalStats = {0.0f, 0.0f, set_tut, 0.0f, 0.0f, 0.0f, rep_count, true};
                updateRepStats(finalStats);

                // Reset all counters for next set
                rep_count = 0;
                set_tut = 0.0f;
                rep_tut = 0.0f;
                set_active = false;
                idle_timing = false;
            }
        } else {
            // Any movement or state change cancels the timeout
            idle_timing = false;
        }
        // ════════════════════════════════════════════════════════

        static uint32_t maxLoopTime = 0;
    uint32_t loopTime = micros() - loopStartMicros;
    if (loopTime > maxLoopTime) maxLoopTime = loopTime;

    
          

        // Report statistics every 0.3 seconds
        if (nowMs - last_report_time >= 300) {
            Serial.print("Hz:"); Serial.print(sample_count * 10 / 3);
            Serial.print(" | Vz:"); Serial.print(vel_z, 2);
            Serial.print(" | [");
            if (zupt_active) Serial.print("ZUPT");
            else if (decay_active) Serial.print("DCAY");
            else Serial.print("FULL");
            Serial.print("] ");
            switch (rep_state) {
                case REP_IDLE:       Serial.print("IDLE"); break;
                case REP_ECCENTRIC:  Serial.print("ECC");  break;
                case REP_CONCENTRIC: Serial.print("CONC"); break;
            }
            Serial.print(" Az:"); Serial.println(acc_earth_z, 2);
            
            sample_count = 0;
            last_report_time = nowMs;
        }
    }
    
    // Note: For maximum speed, avoid adding delays here!
    // The loop will naturally run as fast as the I2C bus allows.
}
// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
/**
 * @brief Print human-readable error message
 */
void printError(int8_t rslt) {
    Serial.print("   Error details: ");
    switch (rslt) {
        case BMI2_E_NULL_PTR:
            Serial.println("Null pointer error");
            break;
        case BMI2_E_COM_FAIL:
            Serial.println("Communication failure - check I2C wiring");
            break;
        case BMI2_E_DEV_NOT_FOUND:
            Serial.println("Device not found - check I2C address");
            break;
        case BMI2_E_INVALID_SENSOR:
            Serial.println("Invalid sensor type specified");
            break;
        case BMI2_E_CONFIG_LOAD:
            Serial.println("Configuration load error");
            break;
        case BMI2_E_INVALID_PAGE:
            Serial.println("Invalid feature page");
            break;
        default:
            Serial.print("Error code: ");
            Serial.println(rslt);
            break;
    }
}
// extra code for error detection
//
// ════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════
/**
 * @brief Halt execution with blinking LED
 */
void haltWithError(const char* message) {
    Serial.println();
    Serial.print("FATAL: ");
    Serial.println(message);
    Serial.println("System halted. Check connections and restart.");
    
    // Blink built-in LED to indicate error
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
}
