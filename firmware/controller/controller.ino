#include <ESP32Servo.h>

constexpr SERVOX_PIN = 32
constexpr SERVOY_PIN = 33

Servo servoX; // changes x acc of ball
Servo servoY; // changes y acc of ball

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
unsigned long ms_1 = 0;

void setup() {
    servoX.attach(SERVOX_PIN, SERVOX_US_MIN, SERVOX_US_MAX);
    servoY.attach(SERVOY_PIN, SERVOY_US_MIN, SERVOY_US_MAX);
    Serial.begin(115200);
    servoX.write(servox_cmd);
    servoY.write(servoy_cmd);
    delay(1000);
}

void loop() {
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');  // read one packet (line)
        line.trim();

        if (sscanf(line.c_str(), "%d,%d", &servox_cmd, &servoy_cmd,) == 2) {
            Serial.print("OK,");
            Serial.print(servox_cmd); Serial.print(',');
            Serial.println(servoy_cmd);

            servox_cmd = constrain(servox_cmd,SERVOX_DEG_MIN,SERVOX_DEG_MAX);
            servoy_cmd = constrain(servoy_cmd,SERVOY_DEG_MIN,SERVOY_DEG_MAX);

            int us_x = map(servox_cmd,SERVOX_DEG_MIN,SERVOX_DEG_MAX, SERVOX_US_MIN, SERVOX_US_MAX);
            int us_y = map(servoy_cmd,SERVOY_DEG_MIN,SERVOY_DEG_MAX, SERVOY_US_MIN, SERVOY_US_MAX);
            
            servoX.writeMicroseconds(us_x);
            servoY.writeMicroseconds(us_y);

            Serial.print()
            
            // unsigned long now = millis();
            // double dt = (now- ms_1) / 1000.0;
            // if (dt >= 0.02) {
            //     // if (abs(x - goal_x) <= goal_window) {
            //     //     x = goal_x;
            //     // }
            //     // if (abs(y - goal_y) <= goal_window) {
            //     //     y = goal_y;
            //     // }
            //     error_x = goal_x - x;
            //     error_y = goal_y - y;
            //     double de_x = (error_x - error_x_1) / dt;
            //     double de_y = (error_y - error_y_1) / dt;

            //     double signal_x = (Kp * error_x) + (Kd * de_x);
            //     double signal_y = (Kp * error_y) + (Kd * de_y);

            //     // Convert to servo angle (center = 90°)
            //     angle_x = constrain(90 - signal_x*S_GAIN, 0, 180);
            //     angle_y = constrain(90 + signal_y*S_GAIN,0, 180);

            //     int us_x = map(angle_x,0,180, X_US_MIN, X_US_MAX);
            //     int us_y = map(angle_y,0,180, Y_US_MIN, Y_US_MAX);
                
            //     servoX.writeMicroseconds(us_x);
            //     servoY.writeMicroseconds(us_y);

            //     // update hist
            //     error_x_1 = error_x;
            //     error_y_1 = error_y;
            //     ms_1 = now;
            // }
        } else {
            Serial.print("BAD,"); Serial.println(line);  // helps debug malformed packets
        }

    }
    // PD

              
}