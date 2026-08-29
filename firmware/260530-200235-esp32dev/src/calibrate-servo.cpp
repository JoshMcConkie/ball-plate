#include "Arduino.h"
#include <ESP32Servo.h>
#include <cmath>
#include "generated_system_config.hpp"

namespace {

Servo servo_a;
Servo servo_b;

enum class Stage {
  DISARMED,
  A_LOW,
  A_HIGH,
  B_LOW,
  B_HIGH,
  COMPLETE,
};

struct EndpointPair {
  int low_us = 0;
  int high_us = 0;
  double angle_at_low_us = NAN;
  double angle_at_high_us = NAN;
};

Stage stage = Stage::DISARMED;
EndpointPair a_points;
EndpointPair b_points;
int a_us = 0;
int b_us = 0;
String input_line;

constexpr int a_midpoint_us() {
  return (system_config::servo_a::calibration_search_min_pulse_us
          + system_config::servo_a::calibration_search_max_pulse_us) / 2;
}

constexpr int b_midpoint_us() {
  return (system_config::servo_b::calibration_search_min_pulse_us
          + system_config::servo_b::calibration_search_max_pulse_us) / 2;
}

const char *stage_name() {
  switch (stage) {
    case Stage::DISARMED: return "DISARMED";
    case Stage::A_LOW: return "A_LOW";
    case Stage::A_HIGH: return "A_HIGH";
    case Stage::B_LOW: return "B_LOW";
    case Stage::B_HIGH: return "B_HIGH";
    case Stage::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

bool is_armed() {
  return stage != Stage::DISARMED;
}

char active_servo() {
  return stage == Stage::A_LOW || stage == Stage::A_HIGH ? 'a' : 'b';
}

int active_min_us() {
  return active_servo() == 'a'
      ? system_config::servo_a::calibration_search_min_pulse_us
      : system_config::servo_b::calibration_search_min_pulse_us;
}

int active_max_us() {
  return active_servo() == 'a'
      ? system_config::servo_a::calibration_search_max_pulse_us
      : system_config::servo_b::calibration_search_max_pulse_us;
}

int &active_pulse_us() {
  return active_servo() == 'a' ? a_us : b_us;
}

void write_positions() {
  servo_a.writeMicroseconds(a_us);
  servo_b.writeMicroseconds(b_us);
}

void center_servos() {
  a_us = a_midpoint_us();
  b_us = b_midpoint_us();
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
  Serial.print(" a ");
  Serial.print(a_us);
  Serial.print(" b ");
  Serial.println(b_us);
}

void print_prompt() {
  if (stage == Stage::A_LOW || stage == Stage::A_HIGH
      || stage == Stage::B_LOW || stage == Stage::B_HIGH) {
    Serial.print("PROMPT ");
    Serial.print(active_servo());
    Serial.print(" ");
    Serial.println(stage == Stage::A_LOW || stage == Stage::B_LOW
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

bool validate_pair(const EndpointPair &points, char servo_id) {
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
    Serial.print("ERROR CENTER_RANGE configured center is outside servo ");
    Serial.print(servo_id);
    Serial.println(" measurements");
    return false;
  }
  return true;
}

void print_result(char servo_id, const EndpointPair &points) {
  const double slope = static_cast<double>(points.high_us - points.low_us)
      / (points.angle_at_high_us - points.angle_at_low_us);
  const double intercept = points.low_us - slope * points.angle_at_low_us;
  const double min_deg = min(points.angle_at_low_us, points.angle_at_high_us);
  const double max_deg = max(points.angle_at_low_us, points.angle_at_high_us);

  Serial.print("RESULT ");
  Serial.print(servo_id);
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
  a_points = EndpointPair{};
  b_points = EndpointPair{};
}

void disarm() {
  if (is_armed()) {
    center_servos();
    delay(250);
    servo_a.detach();
    servo_b.detach();
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

  EndpointPair &points = active_servo() == 'a' ? a_points : b_points;
  if (stage == Stage::A_LOW || stage == Stage::B_LOW) {
    points.low_us = active_pulse_us();
    points.angle_at_low_us = angle_deg;
    stage = stage == Stage::A_LOW ? Stage::A_HIGH : Stage::B_HIGH;
  } else {
    EndpointPair candidate = points;
    candidate.high_us = active_pulse_us();
    candidate.angle_at_high_us = angle_deg;
    if (!validate_pair(candidate, active_servo())) {
      return;
    }
    points = candidate;
    if (stage == Stage::A_HIGH) {
      stage = Stage::B_LOW;
    } else {
      stage = Stage::COMPLETE;
      print_result('a', a_points);
      print_result('b', b_points);
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
    servo_a.attach(
        system_config::servo_a::gpio_pin,
        system_config::servo_a::calibration_search_min_pulse_us,
        system_config::servo_a::calibration_search_max_pulse_us);
    servo_b.attach(
        system_config::servo_b::gpio_pin,
        system_config::servo_b::calibration_search_min_pulse_us,
        system_config::servo_b::calibration_search_max_pulse_us);
    stage = Stage::A_LOW;
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
    stage = Stage::A_LOW;
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
