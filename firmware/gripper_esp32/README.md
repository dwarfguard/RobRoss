# gripper_esp32

Standalone ESP32 firmware that drives an MG996R gripper servo over a plain
serial protocol. This is "Plan B" from the gripper integration discussion:
prove the physical wiring/servo works on its own first, before deciding how
(or whether) it gets wired into `ros2/robross_painter`'s
`painting_executor.cpp`. Nothing here touches the ROS2/Python pipeline.

## Wiring

MG996R has three wires:

| MG996R wire | Connect to |
| --- | --- |
| Signal (orange/yellow) | ESP32 **GPIO18** |
| VCC (red) | A **separate 5-6V power supply**, not the ESP32's own 5V/3.3V pin - MG996R stall current can hit 2-2.5A and will brown out the ESP32's regulator |
| GND (brown/black) | The separate supply's GND, **and** tied together with the ESP32's GND (common ground - without it the signal has no shared reference and the servo will jitter or not respond) |

Before powering anything: confirm the separate supply is 5-6V, confirm all
three grounds (supply, ESP32, servo) are tied together, and confirm the
signal wire only goes to GPIO18.

## Arduino IDE setup

1. **Board support**: Arduino IDE -> Preferences -> "Additional boards manager
   URLs" -> add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`.
   Then Tools -> Board -> Boards Manager -> search `esp32` -> install the
   "esp32 by Espressif Systems" package.
2. **Library**: Tools -> Manage Libraries -> search `ESP32Servo` -> install
   the one by Kevin Harrington / madhephaestus (standard Arduino `Servo.h`
   isn't compatible with the ESP32's PWM peripheral, hence this dedicated
   library).
3. **Board/port**: Tools -> Board -> pick your ESP32 dev board (e.g. "ESP32
   Dev Module"), Tools -> Port -> pick the ESP32's serial port (first plug it
   in over USB; on macOS it usually shows up as
   `/dev/cu.usbserial-XXXX` or `/dev/cu.SLAB_USBtoUART` - installing a
   CP2102/CH340 USB driver first may be needed if it doesn't show up).
4. Open `gripper_esp32.ino` in this folder (the folder name must match the
   `.ino` filename - already set up that way), click Upload.
5. Tools -> Serial Monitor, set baud rate to **115200** (must match
   `Serial.begin(115200)` in the sketch).

## This is a continuous-rotation servo, not a positional one

Despite being sold/labeled as MG996R, this specific unit is continuous
rotation: confirmed with the diagnostic sketch in
`../gripper_esp32_diagnostic/`, which is the plainest possible
`attach()`+`write(angle)` sweep code (copied straight from the esp32io.com
MG996R tutorial) - it rotated the horn through multiple full revolutions
instead of arcing across a bounded 0-180 deg range and holding. A genuine
positional servo physically can't do that regardless of pulse-width
calibration, so this is the unit's real behavior, not a code bug.

For a continuous-rotation servo, `write(value)` doesn't mean "angle" - it
means **speed and direction**: ~90 is the stop point, values below 90 spin
one way (faster the further below 90), values above 90 spin the other way
(faster the further above 90). There's no positional feedback, so the
sketch's "open"/"close" mean "spin at a fixed speed until told to stop", not
"move to a position and hold" - see the file header comment in
`gripper_esp32.ino` for the full protocol.

## Boot homing, and `O`/`H` as timed moves

There's no positional feedback on a continuous-rotation servo, so "open" and
"close" can't be "move to a position and hold" - instead `O` and `H` are
**timed** moves: spin for a fixed duration (`kTimedMoveDurationMs`, measured
by hand at 800ms) and then auto-stop.

- `O` - spin open (`kOpenSpeedValue`) for `kTimedMoveDurationMs`, then stop.
- `H` - spin close (`kCloseSpeedValue`) for the same duration, then stop -
  the reverse of `O`, returning to the original (closed) position.
- `M` - spin close for **half** of `kTimedMoveDurationMs`, then stop - a
  middle position, half as far as `H` (assumes starting from fully open,
  same as `H`).
- On power-up, before accepting any commands, the sketch runs the same
  routine as `O` to home to a known starting position (fully open) - this
  assumes the gripper starts at (or past) fully closed. Watch the Serial
  Monitor for `homing...` followed by `homed` to know when it's done.

If the gripper mechanism changes and 800ms no longer lands at fully
open/closed, re-measure (see "Testing" below) and update
`kTimedMoveDurationMs`.

`C` is kept separate from `H` as a manual, continuous (not timed) close -
spins at `kCloseSpeedValue` until `S` or the watchdog stops it. Useful for
re-measuring the timing by hand without touching the sketch first.

## Testing

In the Serial Monitor:

1. Send `S` at any time - the servo should stop immediately. This is the
   one command to remember first, since anything else may start it spinning.
2. Send `A080` - should spin one direction; send `S` - should stop
   immediately.
3. Send `A120` - should spin the other direction; send `S` - should stop.
4. Send `C` - should spin closed continuously; **deliberately don't send
   `S`** and wait over 2 seconds - the watchdog should auto-stop it (look
   for a `value=90` line printed on its own, without you having sent `S`).
5. Send `O` - should spin open for ~800ms and stop on its own at (roughly)
   fully open.
6. Send `H` - should spin closed for ~800ms and stop on its own, back at
   (roughly) fully closed.
7. Send `O` again, then `M` - should spin closed for ~400ms (half of `H`'s
   duration) and stop roughly halfway between open and closed.
8. Once you've found a speed/direction you're happy with for open/close, use
   `A<value>` to explore further from 90 and confirm those values spin
   cleanly (not straining/stalling) before relying on `O`/`H`/`M`.

To re-measure `kTimedMoveDurationMs` by hand if the gripper mechanism
changes: send `C`, count seconds, send `S` right when it reaches the
position you want, note the time, and update the constant in the sketch.

Also test once with the USB cable unplugged from the computer (ESP32 powered
some other way, e.g. a battery/second USB source) to confirm the firmware
doesn't have a hidden dependency on USB 5V for the logic side - only the
servo's own separate supply should matter for the servo itself.

## Not in scope here

No ROS2/micro-ROS integration, no changes to `ros2/robross_painter` or
`docs/painting-paths-format.md`. Those are separate follow-up work once
there's a decision on where the gripper physically mounts and whether it
reuses the `lower_tool`/`lift_tool` command semantics or gets its own new
command type.
