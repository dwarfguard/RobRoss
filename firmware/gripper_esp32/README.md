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

## Testing

In the Serial Monitor:

1. Send `S` at any time - the servo should stop immediately. This is the
   one command to remember first, since anything else may start it spinning.
2. Send `A080` - should spin one direction; send `S` - should stop
   immediately.
3. Send `A120` - should spin the other direction; send `S` - should stop.
4. Send `O` - should spin; **deliberately don't send `S`** and wait over 2
   seconds - the watchdog should auto-stop it (look for a `value=90` line
   printed on its own, without you having sent `S`).
5. Once you've found a speed/direction you're happy with for "open" and
   "close", use `A<value>` to explore further from 90 and confirm those
   values spin cleanly (not straining/stalling) before relying on `O`/`C`.

There's no positional feedback, so you can't assume `O` always lands the
gripper in the same physical spot - only how long you leave it spinning at a
given speed determines how far it moves. To find how long "fully open" or
"fully closed" actually takes: send `O`, count seconds by hand, send `S`
right when the gripper reaches the position you want, and note the time -
that manual timing is what a future revision would use to make `O`/`C`
auto-stop after a fixed duration instead of requiring a manual `S` (not
implemented yet - see "Not in scope here" below).

Also test once with the USB cable unplugged from the computer (ESP32 powered
some other way, e.g. a battery/second USB source) to confirm the firmware
doesn't have a hidden dependency on USB 5V for the logic side - only the
servo's own separate supply should matter for the servo itself.

## Not in scope here

No auto-timed open/close (`O`/`C` spinning for a fixed duration and stopping
themselves) - that needs the manually-timed open/close duration mentioned
above first, which hasn't been measured yet.

No ROS2/micro-ROS integration, no changes to `ros2/robross_painter` or
`docs/painting-paths-format.md`. Those are separate follow-up work once
there's a decision on where the gripper physically mounts and whether it
reuses the `lower_tool`/`lift_tool` command semantics or gets its own new
command type.
