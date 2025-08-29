#include <Servo.h>

Servo servoX;
Servo servoY;

const double Kp = 1.0;
const double Ki = 0.0;
const double Kd = 0.2;

const double goal_x = 0.0;
const double goal_y = 0.0;

// State
int x=0, y=0, vx=0, vy=0;
String line;

double error_x = 0.0, error_y = 0.0;
double error_x_1 = 0.0, error_y_1 = 0.0;

double de_x = 0.0, de_y = 0.0;

double signal_x = 0, signal_y = 0;

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
    servoX.attach(12);
    servoY.attach(13);
    Serial.begin(115200);
}

void loop() {
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');  // read one packet (line)

        if (sscanf(line.c_str(), "%d,%d,%d,%d", &x, &y, &vx, &vy) == 4) {
            // Successfully parsed all 4 ints
            // Serial.print("X="); Serial.print(x);
            // Serial.print(" Y="); Serial.print(y);
            // Serial.print(" VX="); Serial.print(vx);
            // Serial.print(" VY="); Serial.println(vy);
            error_x = goal_x - x;
            error_y = goal_y - y;   
            signal_x = (Kp * error_x) + (Kd * de_x);
            signal_y = (Kp * error_y) + (Kd * de_y);
        }
    }
    // PD

    unsigned long now = millis();
    double dt = (now- ms_1) / 1000.0;
    if (dt >= 0.02) {
        double de_x = (error_x - error_x_1) / dt;
        double de_y = (error_y - error_y_1) / dt;

        double signal_x = (Kp * err_x) + (Kd * de_x);
        double signal_y = (Kp * err_y) + (Kd * de_y);

        // Convert to servo angle (center = 90°)
        angle_x = constrain(90 + signal_x, MIN_ANG, MAX_ANG);
        angle_y = constrain(90 + signal_y, MIN_ANG, MAX_ANG);

        servoX.write(angle_x);
        servoY.write(angle_y);

        // update hist
        err_x_1 = err_x;
        error_y_1 = error_y;
        ms_1 = now;
    }



    servoX.write(signal_x);
    servoY.write(signal_y);
}