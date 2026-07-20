// ESP32 + continuous-rotation gripper servo (sold/labeled as MG996R, but
// confirmed via a Phase 0 diagnostic - see ../gripper_esp32_diagnostic/ -
// to actually be continuous-rotation: even the plainest possible
// attach()+write(angle) sweep code rotated the horn through multiple full
// revolutions instead of arcing across a bounded 0-180 deg range and
// holding. A genuine positional servo physically cannot do that regardless
// of pulse-width calibration, so this isn't a code bug - it's this unit's
// real behavior. Controlled here over a plain serial protocol (RobRoss
// "Plan B" - no ROS2/micro-ROS involved, just a standalone servo test
// harness so the physical wiring/gripper can be validated on its own
// before anything gets wired into painting_executor.cpp).
//
// For a continuous-rotation servo, the write(angle) call doesn't mean
// "angle" at all - it means "speed and direction": ~90 is the stop point,
// values below 90 spin one way (faster the further below 90), values above
// 90 spin the other way (faster the further above 90). There is no
// positional feedback, so "open"/"close" here means "nudge at a fixed
// speed until told to stop" - not "move to a position and hold".
//
// Protocol (single bytes, read non-blocking so a missing newline never
// stalls the loop):
//   'S'          -> STOP immediately (writes kStopValue). Highest priority -
//                   interrupts an in-progress 'A' digit read too, so it's
//                   always available no matter what state the parser is in.
//   'O'          -> spin at kOpenSpeedValue until 'S' or the watchdog stops it
//   'C'          -> spin at kCloseSpeedValue until 'S' or the watchdog stops it
//   'A' ddd '\n' -> set a raw 0-180 speed value directly (90 = stop) -
//                   digits accumulate until '\n' or a non-digit byte.
// Every accepted command echoes the resulting value back over Serial.
//
// Safety watchdog: if the last commanded value isn't "stop" and no new
// command arrives within kWatchdogTimeoutMs, automatically stop - so a
// dropped serial connection (or a human walking away) never leaves the
// gripper spinning unattended.
//
// Arduino IDE setup: Boards Manager -> install "esp32" (Espressif Systems),
// Library Manager -> install "ESP32Servo" (by Kevin Harrington / madhephaestus).

#include <ESP32Servo.h>

const int kServoPin = 18;
const int kStopValue = 90;
// Carried over from earlier testing - confirmed to spin (not grind/stall)
// at these values. Use 'A<value>' to explore further from 90 if a
// different open/close speed is wanted.
const int kOpenSpeedValue = 60;
const int kCloseSpeedValue = 120;
const unsigned long kWatchdogTimeoutMs = 2000;

Servo gripper;

bool reading_angle = false;
String angle_buffer;
int current_value = kStopValue;
unsigned long last_command_ms = 0;

void SetSpeed(int value) {
  value = constrain(value, 0, 180);
  current_value = value;
  last_command_ms = millis();
  gripper.write(value);
  Serial.print("value=");
  Serial.println(value);
}

void HandleByte(char c) {
  // Stop is highest priority: works even mid-'A' digit entry, so it's
  // always reachable no matter what the parser was doing.
  if (c == 'S') {
    SetSpeed(kStopValue);
    reading_angle = false;
    angle_buffer = "";
    return;
  }

  if (reading_angle) {
    if (isDigit(c)) {
      angle_buffer += c;
      return;
    }
    // Newline or any non-digit ends the value entry.
    if (angle_buffer.length() > 0) {
      SetSpeed(angle_buffer.toInt());
    }
    reading_angle = false;
    angle_buffer = "";
    return;
  }

  switch (c) {
    case 'O':
      SetSpeed(kOpenSpeedValue);
      break;
    case 'C':
      SetSpeed(kCloseSpeedValue);
      break;
    case 'A':
      reading_angle = true;
      angle_buffer = "";
      break;
    default:
      // Ignore whitespace/newlines and any unrecognized byte between commands.
      break;
  }
}

void setup() {
  Serial.begin(115200);
  gripper.attach(kServoPin);  // no custom pulse-width range - library defaults
  SetSpeed(kStopValue);  // boot stopped, not spinning
  Serial.println("gripper ready: send 'S' (stop), 'O', 'C', or 'A<0-180>\\n'");
}

void loop() {
  while (Serial.available() > 0) {
    HandleByte((char)Serial.read());
  }

  if (current_value != kStopValue &&
      millis() - last_command_ms > kWatchdogTimeoutMs) {
    SetSpeed(kStopValue);
  }
}
