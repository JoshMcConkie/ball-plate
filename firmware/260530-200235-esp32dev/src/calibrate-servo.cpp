#include "Arduino.h"
#include <ESP32Servo.h>
#include <cmath>
#include "generated_system_config.hpp"

namespace {

Servo servo_x;
Servo servo_y;

enum class Stage {
  DISARMED,
  X_LOW,
  X_HIGH,
  Y_LOW,
  Y_HIGH,
  COMPLETE,
};

struct EndpointPair {
  int low_us = 0;
  int high_us = 0;
  double angle_at_low_us = NAN;
  double angle_at_high_us = NAN;
};

Stage stage = Stage::DISARMED;
EndpointPair x_points;
EndpointPair y_points;
int x_us = 0;
int y_us = 0;
String input_line;

constexpr int x_midpoint_us() {
  return (system_config::servo_x::calibration_search_min_pulse_us
          + system_config::servo_x::calibration_search_max_pulse_us) / 2;
}

constexpr int y_midpoint_us() {
  return (system_config::servo_y::calibration_search_min_pulse_us
          + system_config::servo_y::calibration_search_max_pulse_us) / 2;
}

const char *stage_name() {
  switch (stage) {
    case Stage::DISARMED: return "DISARMED";
    case Stage::X_LOW: return "X_LOW";
    case Stage::X_HIGH: return "X_HIGH";
    case Stage::Y_LOW: return "Y_LOW";
    case Stage::Y_HIGH: return "Y_HIGH";
    case Stage::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

bool is_armed() {
  return stage != Stage::DISARMED;
}

char active_axis() {
  return stage == Stage::X_LOW || stage == Stage::X_HIGH ? 'x' : 'y';
}

int active_min_us() {
  return active_axis() == 'x'
      ? system_config::servo_x::calibration_search_min_pulse_us
      : system_config::servo_y::calibration_search_min_pulse_us;
}

int active_max_us() {
  return active_axis() == 'x'
      ? system_config::servo_x::calibration_search_max_pulse_us
      : system_config::servo_y::calibration_search_max_pulse_us;
}

int &active_pulse_us() {
  return active_axis() == 'x' ? x_us : y_us;
}

void write_positions() {
  servo_x.writeMicroseconds(x_us);
  servo_y.writeMicroseconds(y_us);
}

void center_servos() {
  x_us = x_midpoint_us();
  y_us = y_midpoint_us();
  if (is_armed()) {
    write_positions();
  }
}

void print_error(const char *code, const char *message) {
  Serial.print("ERROR ");
  Serial.print(code);
  Serial.print(" ");
  Serial.println(message);
}

void print_state() {
  Serial.print("STATE ");
  Serial.print(stage_name());
  Serial.print(" x ");
  Serial.print(x_us);
  Serial.print(" y ");
  Serial.println(y_us);
}

void print_prompt() {
  if (stage == Stage::X_LOW || stage == Stage::X_HIGH
      || stage == Stage::Y_LOW || stage == Stage::Y_HIGH) {
    Serial.print("PROMPT ");
    Serial.print(active_axis());
    Serial.print(" ");
    Serial.println(stage == Stage::X_LOW || stage == Stage::Y_LOW
                       ? "min_pulse"
                       : "max_pulse");
  }
}

bool parse_int_strict(const String &text, int &value) {
  char extra = '\0';
  return sscanf(text.c_str(), "%d %c", &value, &extra) == 1;
}

bool parse_double_strict(const String &text, double &value) {
  char extra = '\0';
  return sscanf(text.c_str(), "%lf %c", &value, &extra) == 1
      && std::isfinite(value);
}

bool validate_pair(const EndpointPair &points, char axis) {
  if (points.low_us >= points.high_us) {
    print_error("PULSE_ORDER", "max-pulse endpoint must exceed min-pulse endpoint");
    return false;
  }
  if (points.angle_at_low_us == points.angle_at_high_us) {
    print_error("ANGLE_RANGE", "endpoint angles must differ");
    return false;
  }
  const double min_deg = min(points.angle_at_low_us, points.angle_at_high_us);
  const double max_deg = max(points.angle_at_low_us, points.angle_at_high_us);
  if (system_config::servo_center_deg < min_deg
      || system_config::servo_center_deg > max_deg) {
    Serial.print("ERROR CENTER_RANGE configured center is outside axis ");
    Serial.print(axis);
    Serial.println(" measurements");
    return false;
  }
  return true;
}

void print_result(char axis, const EndpointPair &points) {
  const double slope = static_cast<double>(points.high_us - points.low_us)
      / (points.angle_at_high_us - points.angle_at_low_us);
  const double intercept = points.low_us - slope * points.angle_at_low_us;
  const double min_deg = min(points.angle_at_low_us, points.angle_at_high_us);
  const double max_deg = max(points.angle_at_low_us, points.angle_at_high_us);

  Serial.print("RESULT ");
  Serial.print(axis);
  Serial.print(" "); Serial.print(points.low_us);
  Serial.print(" "); Serial.print(points.angle_at_low_us, 9);
  Serial.print(" "); Serial.print(points.high_us);
  Serial.print(" "); Serial.print(points.angle_at_high_us, 9);
  Serial.print(" "); Serial.print(min_deg, 9);
  Serial.print(" "); Serial.print(max_deg, 9);
  Serial.print(" "); Serial.print(slope, 9);
  Serial.print(" "); Serial.println(intercept, 9);
}

void reset_measurements() {
  x_points = EndpointPair{};
  y_points = EndpointPair{};
}

void disarm() {
  if (is_armed()) {
    center_servos();
    delay(250);
    servo_x.detach();
    servo_y.detach();
  }
  stage = Stage::DISARMED;
  Serial.println("ACK DISARM");
  print_state();
}

void capture_angle(double angle_deg) {
  if (angle_deg < 0.0 || angle_deg > 180.0) {
    print_error("ANGLE_RANGE", "measured angle must be within 0..180 degrees");
    return;
  }

  EndpointPair &points = active_axis() == 'x' ? x_points : y_points;
  if (stage == Stage::X_LOW || stage == Stage::Y_LOW) {
    points.low_us = active_pulse_us();
    points.angle_at_low_us = angle_deg;
    stage = stage == Stage::X_LOW ? Stage::X_HIGH : Stage::Y_HIGH;
  } else {
    EndpointPair candidate = points;
    candidate.high_us = active_pulse_us();
    candidate.angle_at_high_us = angle_deg;
    if (!validate_pair(candidate, active_axis())) {
      return;
    }
    points = candidate;
    if (stage == Stage::X_HIGH) {
      stage = Stage::Y_LOW;
    } else {
      stage = Stage::COMPLETE;
      print_result('x', x_points);
      print_result('y', y_points);
      center_servos();
      Serial.println("COMPLETE");
    }
  }
  print_state();
  print_prompt();
}

void handle_command(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == "STATUS") {
    print_state();
    print_prompt();
    return;
  }
  if (command == "ARM") {
    if (is_armed()) {
      print_error("STATE", "servos are already armed");
      return;
    }
    reset_measurements();
    center_servos();
    servo_x.attach(
        system_config::servo_x::gpio_pin,
        system_config::servo_x::calibration_search_min_pulse_us,
        system_config::servo_x::calibration_search_max_pulse_us);
    servo_y.attach(
        system_config::servo_y::gpio_pin,
        system_config::servo_y::calibration_search_min_pulse_us,
        system_config::servo_y::calibration_search_max_pulse_us);
    stage = Stage::X_LOW;
    write_positions();
    Serial.println("ACK ARM");
    print_state();
    print_prompt();
    return;
  }
  if (command == "DISARM" || command == "ABORT") {
    disarm();
    return;
  }
  if (!is_armed() || stage == Stage::COMPLETE) {
    print_error("STATE", "ARM or RESET is required before motion commands");
    return;
  }
  if (command == "CENTER") {
    center_servos();
    Serial.println("ACK CENTER");
    print_state();
    return;
  }
  if (command == "RESET") {
    reset_measurements();
    center_servos();
    stage = Stage::X_LOW;
    Serial.println("ACK RESET");
    print_state();
    print_prompt();
    return;
  }

  const int separator = command.indexOf(' ');
  const String verb = separator < 0 ? command : command.substring(0, separator);
  const String argument = separator < 0 ? "" : command.substring(separator + 1);

  if (verb == "SET" || verb == "JOG") {
    int value = 0;
    if (!parse_int_strict(argument, value)) {
      print_error("FORMAT", "SET and JOG require one integer microsecond value");
      return;
    }
    const int requested = verb == "SET" ? value : active_pulse_us() + value;
    if (requested < active_min_us() || requested > active_max_us()) {
      print_error("PULSE_RANGE", "requested pulse is outside the configured search envelope");
      return;
    }
    active_pulse_us() = requested;
    write_positions();
    Serial.println(verb == "SET" ? "ACK SET" : "ACK JOG");
    print_state();
    return;
  }
  if (verb == "CAPTURE") {
    double angle_deg = NAN;
    if (!parse_double_strict(argument, angle_deg)) {
      print_error("FORMAT", "CAPTURE requires one finite angle in degrees");
      return;
    }
    capture_angle(angle_deg);
    return;
  }

  print_error("COMMAND", "unknown command");
}

}  // namespace

void setup() {
  Serial.begin(system_config::serial_baud);
  center_servos();
  Serial.print("READY ");
  Serial.println(system_config::calibration_fingerprint);
  print_state();
}

void loop() {
  while (Serial.available()) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n') {
      handle_command(input_line);
      input_line = "";
    } else if (character != '\r') {
      input_line += character;
      if (input_line.length() > 160) {
        input_line = "";
        print_error("FORMAT", "command exceeds 160 characters");
      }
    }
  }
}
