{\rtf1\ansi\ansicpg1252\cocoartf2822
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 # Barbell Velocity Tracker (VBT) - Arduino Nano 33 BLE Rev2\
\
## Project Overview\
This project aims to develop a high-precision **Velocity Based Training (VBT)** device. Using the **BMI270** 6-axis IMU and **BMM150** magnetometer, the device tracks the vertical displacement and velocity of a barbell during strength training.\
\
## Technical Specifications\
* **Microcontroller:** Arduino Nano 33 BLE Rev2 (Cortex-M4F)\
* **Sensors:** * BMI270 (Accel/Gyro) - 16-bit resolution\
    * BMM150 (Magnetometer) - for heading drift correction\
* **Communication:** Bluetooth Low Energy (BLE) to Web-App\
* **Algorithm:** Madgwick Filter for Sensor Fusion & Orientation Estimation\
\
## Project Roadmap\
1.  **Phase 1:** Hardware Integration & Raw Data Acquisition (I2C)\
2.  **Phase 2:** Sensor Fusion (Madgwick Algorithm) & Gravity Compensation\
3.  **Phase 3:** Velocity Integration & Filtering (Zero-Velocity Update logic)\
4.  **Phase 4:** BLE Data Transmission & Web UI visualization\
\
## How to Use\
1. Clone the repository.\
2. Install `Arduino_BMI270_BMM150` and `MadgwickAHRS` libraries.\
3. Flash the `src/main.ino` to your Arduino.\
\
## License\
This project is licensed under the MIT License - see the LICENSE file for details.}