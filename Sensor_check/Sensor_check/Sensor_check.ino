/*This code it part of Week 1 and had the goal to check the sensor and 
1) read acceelration and gyroscope values
2) timestamping using micros() to measure how many microseconds the loop takes
3) Output data in a format the Serial Plotter likes: label:value
4)Arduino Serial Plotter: this is built into the IDE. You print numbers (Serial.println(accel_x)), and it instantly draws a graph.*/

#include "Arduino_BMI270_BMM150.h"

unsigned long startTime; // Variable to store the start
unsigned long endTime;   // Variable to store the end
unsigned long duration;  // The result

void setup() {
  Serial.begin(115200);
while (!Serial);

  if(!IMU.begin()) {
    Serial.println("failed to inizialize IMU!");
    while(1);
  }
  Serial.println("IMU Inizialized!");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

startTime=micros();

if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
IMU.readAcceleration(ax, ay, az);
IMU.readGyroscope(gx, gy, gz);

endTime=micros();
duration=endTime-startTime;

//Prints acceleration
Serial.print("AX:"); Serial.print(ax); Serial.print(" ");
Serial.print("AY:"); Serial.print(ay); Serial.print(" ");
Serial.print("AZ:"); Serial.print(az); Serial.print(" ");

Serial.print("GX:"); Serial.print(gx); Serial.print(" ");
Serial.print("GY:"); Serial.print(gy); Serial.print(" ");
Serial.print("GZ:"); Serial.print(gz); Serial.print(" ");

Serial.print("LoopTime_us:");
Serial.print(duration);

Serial.println();
}
}