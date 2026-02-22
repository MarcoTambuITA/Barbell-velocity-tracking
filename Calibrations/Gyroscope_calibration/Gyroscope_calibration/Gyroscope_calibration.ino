#include <Arduino_BMI270_BMM150.h>

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0; 
float gx, gy, gz;
float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for you to open the monitor

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
  
  // --- PHASE 1: PREPARATION ---
  Serial.println("--------------------------------");
  Serial.println("PHASE 1: IMU Initialized.");
  Serial.println("Keep the board STILL on the table.");
  Serial.println("Calibration starts in 2 seconds...");
  delay(2000); // Give you time to let go of the board

  // --- PHASE 2: CALIBRATION ---
  Serial.println("PHASE 2: Calibrating (Sampling 2000 times)...");

  // Removed the "if" statement. We enter blindly and wait.
  for (int i = 0; i < 2000; i++) {
    
    // Wait for the sensor to be ready
    while (!IMU.gyroscopeAvailable()); 
    
    IMU.readGyroscope(gx, gy, gz);
    
    sumX += gx;  
    sumY += gy;  
    sumZ += gz;  
  }

  // Calculate Average
  gyroBiasX = sumX / 2000.0;
  gyroBiasY = sumY / 2000.0;
  gyroBiasZ = sumZ / 2000.0;

  // --- PHASE 3: REPORTING ---
  Serial.println("PHASE 3: Calibration Complete.");
  Serial.print("Bias X: "); Serial.println(gyroBiasX);
  Serial.print("Bias Y: "); Serial.println(gyroBiasY);
  Serial.print("Bias Z: "); Serial.println(gyroBiasZ);
  
  Serial.println("--------------------------------");
  Serial.println("Starting Loop in 3 seconds...");
  delay(3000); // The pause you asked for
}

void loop() {
  // We only print inside the loop now
  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(gx, gy, gz);

    // Apply Calibration
    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    // Print
    Serial.print("Gx:"); Serial.print(gx); Serial.print(" ");
    Serial.print("Gy:"); Serial.print(gy); Serial.print(" ");
    Serial.print("Gz:"); Serial.print(gz); Serial.print(" ");

    Serial.println();
  }
}
