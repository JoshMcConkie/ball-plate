#include "Arduino.h"
#include <cmath>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SparkFunLSM6DSO.h>
#include "generated_system_config.hpp"

Servo servoA; // existing mapping: affects x acceleration of ball
Servo servoB; // existing mapping: affects y acceleration of ball
LSM6DSO myIMU;

constexpr int SERVOA_PIN = system_config::servo_a::gpio_pin;
constexpr int SERVOB_PIN = system_config::servo_b::gpio_pin;

constexpr int SERVOA_US_MIN = system_config::servo_a::min_pulse_us;
constexpr int SERVOA_US_MAX = system_config::servo_a::max_pulse_us;

constexpr int SERVOB_US_MIN = system_config::servo_b::min_pulse_us;
constexpr int SERVOB_US_MAX = system_config::servo_b::max_pulse_us;

constexpr int SEND_RATE_HZ = system_config::imu_stream_rate_hz;
constexpr int SEND_PERIOD_MS = 1000 / SEND_RATE_HZ;

constexpr double SERVOA_DEG_MIN = system_config::servo_a::min_deg;
constexpr double SERVOA_DEG_MAX = system_config::servo_a::max_deg;
constexpr double SERVOB_DEG_MIN = system_config::servo_b::min_deg;
constexpr double SERVOB_DEG_MAX = system_config::servo_b::max_deg;

// Servo commands
double servo_a_cmd = system_config::servo_center_deg;
double servo_b_cmd = system_config::servo_center_deg;

// Serial read var
String line;

// timing
unsigned long last_imu_ms = 0;

int servoAMicroseconds(double angle_deg) {
    return constrain(
        static_cast<int>(std::lround(
            system_config::servo_a::deg_to_us_slope * angle_deg
            + system_config::servo_a::deg_to_us_intercept)),
        SERVOA_US_MIN,
        SERVOA_US_MAX);
}

int servoBMicroseconds(double angle_deg) {
    return constrain(
        static_cast<int>(std::lround(
            system_config::servo_b::deg_to_us_slope * angle_deg
            + system_config::servo_b::deg_to_us_intercept)),
        SERVOB_US_MIN,
        SERVOB_US_MAX);
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
    servoA.attach(SERVOA_PIN, SERVOA_US_MIN, SERVOA_US_MAX);
    servoB.attach(SERVOB_PIN, SERVOB_US_MIN, SERVOB_US_MAX);
    
    servoA.writeMicroseconds(servoAMicroseconds(servo_a_cmd));
    servoB.writeMicroseconds(servoBMicroseconds(servo_b_cmd));

    delay(1000);
}

void loop() {
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');  // read one packet (line)
        line.trim();

        if (sscanf(line.c_str(), "%lf,%lf", &servo_a_cmd, &servo_b_cmd) == 2) {
            // Serial.print("OK,");
            // Serial.print(servo_a_cmd); Serial.print(',');
            // Serial.println(servo_b_cmd);

            servo_a_cmd = constrain(servo_a_cmd,SERVOA_DEG_MIN,SERVOA_DEG_MAX);
            servo_b_cmd = constrain(servo_b_cmd,SERVOB_DEG_MIN,SERVOB_DEG_MAX);

            int us_a = servoAMicroseconds(servo_a_cmd);
            int us_b = servoBMicroseconds(servo_b_cmd);
            
            servoA.writeMicroseconds(us_a);
            servoB.writeMicroseconds(us_b);

            
            
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
