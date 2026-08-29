#include "Arduino.h"
#include <cmath>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SparkFunLSM6DSO.h>
#include "generated_system_config.hpp"

Servo servoX; // changes x acc of ball
Servo servoY; // changes y acc of ball
LSM6DSO myIMU;

constexpr int SERVOX_PIN = system_config::servo_x::gpio_pin;
constexpr int SERVOY_PIN = system_config::servo_y::gpio_pin;

constexpr int SERVOX_US_MIN = system_config::servo_x::min_pulse_us;
constexpr int SERVOX_US_MAX = system_config::servo_x::max_pulse_us;

constexpr int SERVOY_US_MIN = system_config::servo_y::min_pulse_us;
constexpr int SERVOY_US_MAX = system_config::servo_y::max_pulse_us;

constexpr int SEND_RATE_HZ = system_config::imu_stream_rate_hz;
constexpr int SEND_PERIOD_MS = 1000 / SEND_RATE_HZ;

constexpr double SERVOX_DEG_MIN = system_config::servo_x::min_deg;
constexpr double SERVOX_DEG_MAX = system_config::servo_x::max_deg;
constexpr double SERVOY_DEG_MIN = system_config::servo_y::min_deg;
constexpr double SERVOY_DEG_MAX = system_config::servo_y::max_deg;

// Servo commands
double servox_cmd = system_config::servo_center_deg;
double servoy_cmd = system_config::servo_center_deg;

// Serial read var
String line;

// timing
unsigned long last_imu_ms = 0;

int servoXMicroseconds(double angle_deg) {
    return constrain(
        static_cast<int>(std::lround(
            system_config::servo_x::deg_to_us_slope * angle_deg
            + system_config::servo_x::deg_to_us_intercept)),
        SERVOX_US_MIN,
        SERVOX_US_MAX);
}

int servoYMicroseconds(double angle_deg) {
    return constrain(
        static_cast<int>(std::lround(
            system_config::servo_y::deg_to_us_slope * angle_deg
            + system_config::servo_y::deg_to_us_intercept)),
        SERVOY_US_MIN,
        SERVOY_US_MAX);
}

void setup() {
    Serial.begin(system_config::serial_baud);
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
    
    servoX.writeMicroseconds(servoXMicroseconds(servox_cmd));
    servoY.writeMicroseconds(servoYMicroseconds(servoy_cmd));

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

            int us_x = servoXMicroseconds(servox_cmd);
            int us_y = servoYMicroseconds(servoy_cmd);
            
            servoX.writeMicroseconds(us_x);
            servoY.writeMicroseconds(us_y);

            
            
        } else {
            // Serial.print("BAD,"); Serial.println(line);  // helps debug malformed packets
        }
    }

    if (millis() - last_imu_ms >= SEND_PERIOD_MS) {
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
