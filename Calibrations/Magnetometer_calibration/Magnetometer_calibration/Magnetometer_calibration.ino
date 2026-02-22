#include <Arduino_BMI270_BMM150.h>

float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("MAGNETOMETER CALIBRATION");
  Serial.println("--------------------------------");
  Serial.println("In 2 seconds, start rotating the board in a FIGURE-8 motion.");
  Serial.println("Turn it upside down, sideways, everywhere!");
  delay(2000);
  Serial.println("START ROTATING NOW! (15 seconds)");

  // Initialize variables with extreme opposite values
  float xMin = 9999, xMax = -9999;
  float yMin = 9999, yMax = -9999;
  float zMin = 9999, zMax = -9999;
  
  unsigned long startCal = millis();
  
  // Run for 15 seconds (15000 ms)
  while (millis() - startCal < 15000) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);

      // Check for new Max/Min
      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;

      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;
    }
  }

  // Calculate the Offsets (Hard Iron Correction)
  magOffsetX = (xMax + xMin) / 2;
  magOffsetY = (yMax + yMin) / 2;
  magOffsetZ = (zMax + zMin) / 2;

  Serial.println("--------------------------------");
  Serial.println("Calibration Complete.");
  Serial.print("Offset X: "); Serial.println(magOffsetX);
  Serial.print("Offset Y: "); Serial.println(magOffsetY);
  Serial.print("Offset Z: "); Serial.println(magOffsetZ);
  Serial.println("--------------------------------");
  Serial.println("You can now paste these values into your main code.");
  delay(5000);
}

void loop() {
  // Test the calibration
  if (IMU.magneticFieldAvailable()) {
    float x, y, z;
    IMU.readMagneticField(x, y, z);

    // Apply the offset
    x = x - magOffsetX;
    y = y - magOffsetY;
    z = z - magOffsetZ;

    Serial.print("Mx:"); Serial.print(x); Serial.print(" ");
    Serial.print("My:"); Serial.print(y); Serial.print(" ");
    Serial.print("Mz:"); Serial.print(z); 
    Serial.println();
  }
}