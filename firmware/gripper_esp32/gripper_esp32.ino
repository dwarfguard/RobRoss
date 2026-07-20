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
//   'O'          -> timed move: spin open for kTimedMoveDurationMs, then
//                   auto-stop. Same routine boot uses to home the gripper.
//   'H'          -> timed move: spin close for kTimedMoveDurationMs (same
//                   duration as 'O'), then auto-stop - the reverse of 'O',
//                   returning to the original (closed) position.
//   'M'          -> timed move: spin close for half of kTimedMoveDurationMs,
//                   then auto-stop - a middle position, half as far as 'H'
//                   (assumes starting from fully open, same as 'H' does).
//   'C'          -> spin at kCloseSpeedValue until 'S' or the watchdog stops
//                   it (manual/continuous - kept for re-calibrating timing,
//                   e.g. if kTimedMoveDurationMs needs to change).
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
const int kOpenSpeedValue = 120;
const int kCloseSpeedValue = 60;
const unsigned long kWatchdogTimeoutMs = 2000;
// Measured by hand: how long spinning at kOpenSpeedValue takes to reach
// fully open from a fully-closed start. Used for both 'O' (open) and 'H'
// (close, the reverse) since the open/close travel is assumed symmetric -
// re-measure and update this if the gripper mechanism changes.
const unsigned long kTimedMoveDurationMs = 800;

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

// Spins at speed_value for duration_ms, then auto-stops. Used for 'O'
// (open), 'H' (close/return), 'M' (middle, half duration), and boot homing -
// all the same shape of move, just with a different speed_value/duration_ms
// and different log text.
void TimedMove(int speed_value, unsigned long duration_ms, const char *start_label, const char *done_label) {
  Serial.println(start_label);
  SetSpeed(speed_value);
  delay(duration_ms);
  SetSpeed(kStopValue);
  Serial.println(done_label);
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
      TimedMove(kOpenSpeedValue, kTimedMoveDurationMs, "opening...", "opened");
      break;
    case 'H':
      TimedMove(kCloseSpeedValue, kTimedMoveDurationMs, "returning...", "returned");
      break;
    case 'M':
      TimedMove(kCloseSpeedValue, kTimedMoveDurationMs / 2, "closing to middle...", "at middle");
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

  // Known starting position before accepting commands - same routine as 'O'.
  TimedMove(kOpenSpeedValue, kTimedMoveDurationMs, "homing...", "homed");

  Serial.println("gripper ready: send 'S' (stop), 'O', 'H', 'M', 'C', or 'A<0-180>\\n'");
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
