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
// Claw starts physically closed (default resting position when power/
// serial is first connected). setup() then homes it open before accepting
// commands, so by the time "gripper ready" prints, the claw is sitting
// fully open.
//
// Protocol (single bytes, read non-blocking so a missing newline never
// stalls the loop):
//   'S'  -> STOP immediately (writes kStopValue). Highest priority -
//           interrupts an in-progress 'A' digit read too, so it's always
//           available no matter what state the parser is in.
//   'O'  -> timed move: spin open for kTimedMoveDurationMs * 69/48 (longer
//           than one full open/close travel), then auto-stop. Deliberately
//           overshoots a plain "open" so the marker pops loose - meant to
//           be run from the 'G' (grabbed) position.
//   'G'  -> timed move: spin close for kTimedMoveDurationMs * 20/24, then
//           auto-stop - closes the claw enough to grab/secure the marker.
//           Assumes starting from the fully-open (homed) position.
//   'R'  -> timed move: spin close for kTimedMoveDurationMs * 6/24 (a short
//           nudge), then auto-stop. Run this after 'O' to bring the claw
//           back from its overshot-open position to the same fully-open
//           position 'G' expects to start from, so 'G' can be called again.
//   'C'  -> spin at kCloseSpeedValue until 'S' or the watchdog stops it
//           (manual/continuous - kept for re-calibrating timing, e.g. if
//           kTimedMoveDurationMs needs to change).
//   'A' ddd '\n' -> set a raw 0-180 speed value directly (90 = stop) -
//           digits accumulate until '\n' or a non-digit byte.
// Every accepted command echoes the resulting value back over Serial.
//
// Safety watchdog: if the last commanded value isn't "stop" and no new
// command arrives within kWatchdogTimeoutMs, automatically stop - so a
// dropped serial connection (or a human walking away) never leaves the
// gripper spinning unattended.
//
// RS485: the same protocol above is also accepted over a second, hardware
// UART (Serial2) wired through a TTL-to-RS485 transceiver, so the robot's
// controller can issue commands over the RS485 bus instead of (or as well
// as) USB. USB stays live as a debug/manual-control port; RS485 is the
// intended production control path. Both feed the same HandleByte()
// parser and share the same watchdog, so a command from either source
// behaves identically. RS485 is half-duplex: kRs485DePin drives the
// transceiver's DE+RE (tied together) - HIGH to transmit, LOW to receive
// - and the line is only driven HIGH briefly, via Rs485Reply(), to send a
// one-line ack after a complete command arrives over Serial2. Wiring:
// transceiver TXD -> kRs485RxPin, transceiver RXD -> kRs485TxPin (cross-
// connected, as with any UART peripheral), transceiver DE+RE ->
// kRs485DePin. See ../gripper_esp32/README.md for the full wiring table.
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
// Measured by hand: how long spinning at kOpenSpeedValue/kCloseSpeedValue
// takes to travel the full closed<->open range. Used as the base unit for
// 'O', 'G', and 'R', each of which scales it by its own fraction (see
// the switch statement below) rather than using it unscaled.
const unsigned long kTimedMoveDurationMs = 470;

// RS485 (Serial2/UART2) pins - all currently-free GPIOs, no conflict with
// kServoPin. Must match whatever baud the robot controller's RS485 port
// is configured for; change kRs485Baud if it isn't 115200.
const int kRs485RxPin = 16;
const int kRs485TxPin = 17;
const int kRs485DePin = 21;
const unsigned long kRs485Baud = 115200;

Servo gripper;

bool reading_angle = false;
String angle_buffer;
int current_value = kStopValue;
unsigned long last_command_ms = 0;

// Drives the RS485 transceiver's DE+RE pin: HIGH puts it in transmit
// mode (driver enabled, receiver disabled), LOW puts it back in receive
// mode. The short delay lets the transceiver chip actually switch before
// bytes start moving either direction.
void Rs485SetTransmit(bool enable) {
  digitalWrite(kRs485DePin, enable ? HIGH : LOW);
  delayMicroseconds(10);
}

// Sends msg out over RS485. Half-duplex, so the bus must be released back
// to receive mode as soon as the bytes are actually sent - Serial2.flush()
// blocks until they've physically finished shifting out, so switching
// back to receive mode right after doesn't clip the last byte.
void Rs485Reply(const char *msg) {
  Rs485SetTransmit(true);
  Serial2.print(msg);
  Serial2.flush();
  Rs485SetTransmit(false);
}

void SetSpeed(int value) {
  value = constrain(value, 0, 180);
  current_value = value;
  last_command_ms = millis();
  gripper.write(value);
  Serial.print("value=");
  Serial.println(value);
}

// Spins at speed_value for duration_ms, then auto-stops. Used for 'O'
// (open/overshoot to release), 'G' (close to grab), 'R' (short return
// nudge), and boot homing - all the same shape of move, just with a
// different speed_value/duration_ms and different log text.
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
      TimedMove(kOpenSpeedValue, kTimedMoveDurationMs * 69 / 48, "opening to release pen...", "opened");
      break;
    case 'G':
      TimedMove(kCloseSpeedValue, kTimedMoveDurationMs * 20 / 24, "closing to grab pen...", "closed");
      break;
    case 'R':
      TimedMove(kCloseSpeedValue, kTimedMoveDurationMs * 6 / 24, "returning to grab-ready position...", "ready");
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

  Serial2.begin(kRs485Baud, SERIAL_8N1, kRs485RxPin, kRs485TxPin);
  pinMode(kRs485DePin, OUTPUT);
  Rs485SetTransmit(false);  // default to receive mode

  gripper.attach(kServoPin);  // no custom pulse-width range - library defaults

  // Claw starts physically closed. Home it fully open before accepting
  // commands so 'G' always has a known, consistent starting position.
  TimedMove(kOpenSpeedValue, kTimedMoveDurationMs, "homing...", "homed");

  Serial.println("gripper ready: send 'S' (stop), 'O', 'G', 'R', 'C', or 'A<0-180>\\n' over USB or RS485");
}

void loop() {
  while (Serial.available() > 0) {
    HandleByte((char)Serial.read());
  }

  // Same parser, fed from the RS485 UART. After a complete command (not
  // still mid-digit-entry for 'A') reply once with the resulting speed
  // value, so the robot controller can confirm the command took effect.
  bool rs485_had_data = false;
  while (Serial2.available() > 0) {
    HandleByte((char)Serial2.read());
    rs485_had_data = true;
  }
  if (rs485_had_data && !reading_angle) {
    char reply[16];
    snprintf(reply, sizeof(reply), "value=%d\n", current_value);
    Rs485Reply(reply);
  }

  if (current_value != kStopValue &&
      millis() - last_command_ms > kWatchdogTimeoutMs) {
    SetSpeed(kStopValue);
  }
}
