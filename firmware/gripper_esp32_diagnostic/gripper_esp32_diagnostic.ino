// Phase 0 diagnostic sketch - NOT the real gripper firmware (see
// ../gripper_esp32/ for that). This is deliberately the plainest possible
// servo test: no custom pulse-width range, no protocol, just attach() and a
// slow full sweep, copied from the esp32io.com MG996R tutorial's own
// example (only the pin changed from 9 to 18 to match our existing wiring).
//
// Purpose: find out whether the "keeps spinning instead of arriving and
// holding" symptom seen with the real firmware's custom pulse-width range
// (600-2400us) is caused by that custom range, or is inherent to this
// specific servo unit regardless of code.
//
// What to watch for after uploading:
//   - Sweeps smoothly 0 -> 180 -> 0 and settles/holds at each end
//     (even if the exact end angle looks a little off - any clean stop
//     counts) -> the servo is a normal positional unit; the bug is in the
//     real firmware's custom pulse-width setup. See Phase 1a in the plan.
//   - Instead it's basically stationary only around the middle of the sweep
//     and spends most of the sweep spinning continuously, with speed
//     tracking how far the loop's "angle" variable is from 90 -> this is a
//     continuous-rotation unit regardless of what it's labeled. See
//     Phase 1b in the plan.

#include <ESP32Servo.h>

Servo servo;

void setup() {
  servo.attach(18);  // no custom pulse-width range - library defaults
  servo.write(0);
}

void loop() {
  for (int angle = 0; angle <= 180; angle += 1) {
    servo.write(angle);
    delay(10);
  }
  delay(500);
  for (int angle = 180; angle >= 0; angle -= 1) {
    servo.write(angle);
    delay(10);
  }
  delay(500);
}
