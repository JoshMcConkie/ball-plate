#include <Servo.h>

Servo servoX;
Servo servoY;

const double S_GAIN = 1.0;

const double Kp = 0.5;
const double Ki = 0.05;
const double Kd = 0.1;

const double goal_x = 0.0;
const double goal_y = 0.0;

const double goal_window = 10;

// State
int vx=0, vy=0;
int x=0, y=0;
String line;

double error_x = 0.0, error_y = 0.0;
double error_x_1 = 0.0, error_y_1 = 0.0;

// timing
unsigned long ms_1 = 0;

// Servo outs
double angle_x =90.0, angle_y = 90.0;

// servo constraints
const double MIN_ANG_x = 60.0; 
const double MAX_ANG_x = 120.0;

const double MIN_ANG_y = 60.0; 
const double MAX_ANG_y = 120.0;


void setup() {
    servoX.attach(9, 600, 2400);
    servoY.attach(10, 600, 2400);
    Serial.begin(115200);
    servoX.write(angle_x);
    servoY.write(angle_y);
    delay(1000);
}

void loop() {
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');  // read one packet (line)
        line.trim();

        if (sscanf(line.c_str(), "%d,%d,%d,%d", &x, &y, &vx, &vy) == 4) {
            Serial.print("OK,");
            Serial.print(x); Serial.print(',');
            Serial.print(y); Serial.print(',');
            Serial.print(vx); Serial.print(',');
            Serial.println(vy);
            // Successfully parsed all 4 ints
            // Serial.print("X="); Serial.print(x);
            // Serial.print(" Y="); Serial.print(y);
            // Serial.print(" VX="); Serial.print(vx);
            // Serial.print(" VY="); Serial.println(vy);
            unsigned long now = millis();
            double dt = (now- ms_1) / 1000.0;
            if (dt >= 0.02) {
                if (abs(x - goal_x) <= goal_window) {
                    x = goal_x;
                }
                if (abs(y - goal_y) <= goal_window) {
                    y = goal_y;
                }
                error_x = goal_x - x;
                error_y = goal_y - y;
                double de_x = (error_x - error_x_1) / dt;
                double de_y = (error_y - error_y_1) / dt;

                double signal_x = (Kp * error_x) + (Kd * de_x);
                double signal_y = (Kp * error_y) + (Kd * de_y);

                // Convert to servo angle (center = 90°)
                angle_x = constrain(90 + signal_x*S_GAIN, MIN_ANG_x, MAX_ANG_x);
                angle_y = constrain(90 + signal_y*S_GAIN, MIN_ANG_y, MAX_ANG_y);

                servoX.write(180-angle_x);
                servoY.write(angle_y);

                // update hist
                error_x_1 = error_x;
                error_y_1 = error_y;
                ms_1 = now;
            }
        } else {
            Serial.print("BAD,"); Serial.println(line);  // helps debug malformed packets
        }

    }
    // PD

              
}