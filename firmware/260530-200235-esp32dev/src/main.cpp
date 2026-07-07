#include "Arduino.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include <SparkFunLSM6DSO.h>

constexpr int SERVOX_PIN = 32;
constexpr int SERVOY_PIN = 33;

Servo servoX; // changes x acc of ball
Servo servoY; // changes y acc of ball
LSM6DSO myIMU;

// servo constraints
constexpr int SERVOX_US_MIN = 800, SERVOY_US_MIN = 800;
constexpr int SERVOX_US_MAX = 2200, SERVOY_US_MAX = 2200;

constexpr double SERVOX_DEG_MIN = 60.0, SERVOY_DEG_MIN = 60.0;
constexpr double SERVOX_DEG_MAX = 120.0, SERVOY_DEG_MAX = 120.0;

// Servo commands
double servox_cmd = 90.0, servoy_cmd = 90.0;

// IMU state
double roll = 0, pitch = 0;

// Serial read var
String line;

// timing
unsigned long last_imu_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Booting...");

    // IMU init
    Wire.begin(21,22,100000);
    delay(10);

    if( myIMU.begin(0x6B, Wire) )
        Serial.println("Ready.");
    else { 
        Serial.println("Could not connect to IMU.");
        Serial.println("Freezing");
    }

    if( myIMU.initialize(BASIC_SETTINGS) )
        Serial.println("Loaded Settings.");

    // Servo init
    servoX.attach(SERVOX_PIN, SERVOX_US_MIN, SERVOX_US_MAX);
    servoY.attach(SERVOY_PIN, SERVOY_US_MIN, SERVOY_US_MAX);
    
    servoX.write(servox_cmd);
    servoY.write(servoy_cmd);

    delay(1000);
}

void loop() {
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');  // read one packet (line)
        line.trim();

        if (sscanf(line.c_str(), "%lf,%lf", &servox_cmd, &servoy_cmd) == 2) {
            // Serial.print("OK,");
            // Serial.print(servox_cmd); Serial.print(',');
            // Serial.println(servoy_cmd);

            servox_cmd = constrain(servox_cmd,SERVOX_DEG_MIN,SERVOX_DEG_MAX);
            servoy_cmd = constrain(servoy_cmd,SERVOY_DEG_MIN,SERVOY_DEG_MAX);

            // Map on the full 0-180 deg scale so a degree command maps to its
            // true position (90 deg -> center). The constrain above limits travel.
            int us_x = map(servox_cmd, 0.0, 180.0, SERVOX_US_MIN, SERVOX_US_MAX);
            int us_y = map(servoy_cmd, 0.0, 180.0, SERVOY_US_MIN, SERVOY_US_MAX);
            
            servoX.writeMicroseconds(us_x);
            servoY.writeMicroseconds(us_y);

            
            
        } else {
            // Serial.print("BAD,"); Serial.println(line);  // helps debug malformed packets
        }
    }

    if (millis() - last_imu_ms >= 50) {
        last_imu_ms = millis();

        // Send IMU state packet: ax ay az gx gy gz
        Serial.print(myIMU.readFloatAccelX(), 3);Serial.print(" ");
        Serial.print(myIMU.readFloatAccelY(), 3);Serial.print(" ");
        Serial.print(myIMU.readFloatAccelZ(), 3);Serial.print(" ");
        Serial.print(myIMU.readFloatGyroX(), 3);Serial.print(" ");
        Serial.print(myIMU.readFloatGyroY(), 3);Serial.print(" ");
        Serial.println(myIMU.readFloatGyroZ(), 3);
    }
}
