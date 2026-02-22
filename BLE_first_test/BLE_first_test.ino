#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <math.h>

// -------------------------------------------------------------------------
// 1. CONFIGURATION & GLOBALS
// -------------------------------------------------------------------------

// BLE UUIDs (Must match your Website/Antigravity code)
BLEService vbtService("19B10000-E8F2-537E-4F6C-D104768A1214");
// Characteristic: Notify, 28 bytes (7 floats * 4 bytes)
BLECharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 28); 

// YOUR CALIBRATION OFFSETS (Enter your values here)
float mag_off_x = 0.0; 
float mag_off_y = 1.5;
float mag_off_z = -23.5;

// Madgwick Variables (Global - accessible everywhere!)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
float beta = 0.5f;   // Gain: 0.1 (Smooth) to 1.0 (Responsive)
float sampleFreq = 100.0f; 

// -------------------------------------------------------------------------
// 2. SETUP
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  if (!IMU.begin()) { Serial.println("IMU Failed"); while (1); }
  if (!BLE.begin()) { Serial.println("BLE Failed"); while (1); }

  BLE.setLocalName("VBT_Sensor");
  BLE.setAdvertisedService(vbtService);
  vbtService.addCharacteristic(dataChar);
  BLE.addService(vbtService);
  BLE.advertise();
  
  Serial.println("System Ready. Waiting for Web Bluetooth...");
}

// -------------------------------------------------------------------------
// 3. MAIN LOOP
// -------------------------------------------------------------------------
void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to: "); Serial.println(central.address());
    
    while (central.connected()) {
      static unsigned long lastUpdate = 0;
      unsigned long now = millis();

      // Run loop at 100Hz (10ms)
      if (now - lastUpdate >= 10) {
        lastUpdate = now;
        
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
          IMU.readAcceleration(ax, ay, az);
          IMU.readGyroscope(gx, gy, gz);
          IMU.readMagneticField(mx, my, mz);

          // A. Apply Calibration
          mx -= mag_off_x;
          my -= mag_off_y;
          mz -= mag_off_z;

          // B. Convert Gyro to Radians/Sec (Math requires radians)
          float gx_rad = gx * DEG_TO_RAD;
          float gy_rad = gy * DEG_TO_RAD;
          float gz_rad = gz * DEG_TO_RAD;

          // C. Run Sensor Fusion
          // This updates the global q0, q1, q2, q3 variables directly
          MadgwickQuaternionUpdate(ax, ay, az, gx_rad, gy_rad, gz_rad, mx, my, mz);

          // D. Create Data Packet (Quaternion + Raw Acceleration)
          // We send Raw Accel so the Website can subtract gravity accurately in 3D space
          float dataPacket[7];
          dataPacket[0] = q0;
          dataPacket[1] = q1;
          dataPacket[2] = q2;
          dataPacket[3] = q3;
          dataPacket[4] = ax; // Send raw X
          dataPacket[5] = ay; // Send raw Y
          dataPacket[6] = az; // Send raw Z

          // E. Send via BLE
          dataChar.writeValue((byte*)dataPacket, 28);
        }
      }
    }
    Serial.println("Disconnected");
  }
}

// -------------------------------------------------------------------------
// 4. THE MATH ENGINE (Madgwick Algorithm)
// -------------------------------------------------------------------------
// This replaces the library. It fuses the sensors into the quaternion.
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q1x = q0, q1y = q1, q1z = q2, q1w = q3;
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables
  float _2q1mx, _2q1my, _2q1mz, _2q2mx, _4bx, _4bz;
  float _2q1 = 2.0f * q1x;
  float _2q2 = 2.0f * q1y;
  float _2q3 = 2.0f * q1z;
  float _2q4 = 2.0f * q1w;
  float _2q1q3 = 2.0f * q1x * q1z;
  float _2q3q4 = 2.0f * q1z * q1w;
  float q1q1 = q1x * q1x;
  float q1q2 = q1x * q1y;
  float q1q3 = q1x * q1z;
  float q1q4 = q1x * q1w;
  float q2q2 = q1y * q1y;
  float q2q3 = q1y * q1z;
  float q2q4 = q1y * q1w;
  float q3q3 = q1z * q1z;
  float q3q4 = q1z * q1w;
  float q4q4 = q1w * q1w;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; 
  norm = 1.0f / norm;
  ax *= norm; ay *= norm; az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; 
  norm = 1.0f / norm;
  mx *= norm; my *= norm; mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1x * mx;
  _2q1my = 2.0f * q1x * my;
  _2q1mz = 2.0f * q1x * mz;
  _2q2mx = 2.0f * q1y * mx;
  hx = mx * q1q1 - _2q1my * q1y - _2q1mz * q1z + _2q2mx * q1y + mx * q2q2 + _2q3 * my * q1w - _2q4 * my * q1z + mz * q3q3 + _2q4 * mz * q1y - mz * q4q4;
  hy = _2q1mx * q1y + my * q1q1 - _2q1mz * q1w + _2q2mx * q1z - mx * q2q2 + my * q2q2 + _2q4 * mz * q1x - mz * q3q3 + _2q3 * mx * q1w - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q1z + _2q1my * q1w + mz * q1q1 + _2q2mx * q1w - mz * q2q2 + _2q3 * my * q1z - mz * q3q3 + mx * q4q4 - my * q4q4 - mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient descent algorithm corrective step
  s1 = -_2q3 * (2.0f * q1y * q1w - _2q1q3 - ax) + _2q2 * (2.0f * q1x * q1y + _2q3q4 - ay) - _2bz * q1z * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q1z * q1w - q1x * q1y) - mx) + (-_2bx * q1w + _2bz * q1y) * (_2bx * (q1y * q1z - q1x * q1w) + _2bz * (q1x * q1z + q1y * q1w) - my) + _2bx * q1z * (_2bx * (q1x * q1z + q1y * q1w) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q1y * q1w - _2q1q3 - ax) + _2q1 * (2.0f * q1x * q1y + _2q3q4 - ay) - 4.0f * q1y * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q1w * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q1z * q1w - q1x * q1y) - mx) + (_2bx * q1z + _2bz * q1x) * (_2bx * (q1y * q1z - q1x * q1w) + _2bz * (q1x * q1z + q1y * q1w) - my) + (_2bx * q1w - _4bz * q1y) * (_2bx * (q1x * q1z + q1y * q1w) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q1y * q1w - _2q1q3 - ax) + _2q4 * (2.0f * q1x * q1y + _2q3q4 - ay) - 4.0f * q1z * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q1z - _2bz * q1x) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q1z * q1w - q1x * q1y) - mx) + (_2bx * q1y + _2bz * q1w) * (_2bx * (q1y * q1z - q1x * q1w) + _2bz * (q1x * q1z + q1y * q1w) - my) + (_2bx * q1x - _4bz * q1z) * (_2bx * (q1x * q1z + q1y * q1w) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q1y * q1w - _2q1q3 - ax) + _2q3 * (2.0f * q1x * q1y + _2q3q4 - ay) + (-_4bx * q1w + _2bz * q1y) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q1z * q1w - q1x * q1y) - mx) + (-_2bx * q1x + _2bz * q1z) * (_2bx * (q1y * q1z - q1x * q1w) + _2bz * (q1x * q1z + q1y * q1w) - my) + _2bx * q1y * (_2bx * (q1x * q1z + q1y * q1w) + _2bz * (0.5f - q2q2 - q3q3) - mz);

  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    
  norm = 1.0f / norm;
  s1 *= norm; s2 *= norm; s3 *= norm; s4 *= norm;

  // Rate of change
  qDot1 = 0.5f * (-q1y * gx - q1z * gy - q1w * gz) - beta * s1;
  qDot2 = 0.5f * (q1x * gx + q1z * gz - q1w * gy) - beta * s2;
  qDot3 = 0.5f * (q1x * gy - q1y * gz + q1w * gx) - beta * s3;
  qDot4 = 0.5f * (q1x * gz + q1y * gy - q1z * gx) - beta * s4;

  // Integrate
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  norm = 1.0f / norm;
  q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}