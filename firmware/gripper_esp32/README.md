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

### RS485 (robot controller control port)

USB stays connected as a debug/manual-control port (Serial Monitor still
works exactly as below); RS485 is a second, independent input wired
through a TTL-to-RS485 transceiver module (the common MAX485 breakout,
6 pins labeled `TXD`/`RXD`/`DE`/`RE`/`A`/`B`) so the robot's controller can
send the same commands over a two-wire differential bus instead of USB.
Both inputs run the same parser and share the same watchdog - a command
from either source behaves identically, and either can be used at the same
time.

| Transceiver pin | Connect to | Notes |
| --- | --- | --- |
| `TXD` (module transmits, i.e. `RO`) | ESP32 **GPIO16** (`Serial2` RX) | Cross-connected, as with any UART peripheral |
| `RXD` (module receives, i.e. `DI`) | ESP32 **GPIO17** (`Serial2` TX) | |
| `DE` **and** `RE` (tied together) | ESP32 **GPIO21** | Firmware drives this HIGH only briefly to send a reply, LOW (receive) the rest of the time |
| `VCC` | 3.3V, **only if the module's logic side is 3.3V** | **Confirm before wiring** - ESP32 GPIOs are not 5V-tolerant. If the module's `TXD`/`RXD`/`DE`/`RE` pins are 5V logic, either use a 3.3V-logic module (e.g. MAX3485-based) or add a 4-channel logic-level shifter between the ESP32 and the module instead of wiring directly |
| `GND` | ESP32 GND, common with the servo supply's GND | |
| `A` / `B` (or `D+`/`D-`) | Robot controller's RS485 port, via twisted pair | Add a 120Ω termination resistor across A/B at each end of the bus if the run is long or noisy |

GPIO16/17/21 are all free on this board (only GPIO18 is claimed, by the
servo signal). Baud rate defaults to 115200 (`kRs485Baud` in the sketch) -
must match whatever the robot controller's RS485 port is configured for;
change the constant and re-flash if it isn't 115200.

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

## Boot homing, and `O`/`G`/`R` as timed moves

There's no positional feedback on a continuous-rotation servo, so "open" and
"close" can't be "move to a position and hold" - instead `O`, `G`, and `R`
are **timed** moves: spin for a fixed duration (scaled from
`kTimedMoveDurationMs`, measured by hand at 470ms) and then auto-stop.

- `G` - spin close (`kCloseSpeedValue`) to grab/secure the pen. Assumes
  starting from the fully-open (homed) position.
- `O` - spin open (`kOpenSpeedValue`), deliberately overshooting a plain
  "open" so the pen pops loose. Meant to be run from the `G` (grabbed)
  position.
- `R` - a short close nudge, bringing the claw back from `O`'s overshot-open
  position to the same fully-open position `G` expects to start from, so
  `G` can be called again.
- On power-up, before accepting any commands, the sketch runs the same
  routine as `O` to home to a known starting position (fully open) - this
  assumes the gripper starts at (or past) fully closed. Watch the Serial
  Monitor for `homing...` followed by `homed` to know when it's done.

If the gripper mechanism changes and these durations no longer land at the
right positions, re-measure (see "Testing" below) and update
`kTimedMoveDurationMs` and/or the per-command fractions in `HandleByte()`.

`C` is kept separate from `G`/`O`/`R` as a manual, continuous (not timed)
close - spins at `kCloseSpeedValue` until `S` or the watchdog stops it.
Useful for re-measuring the timing by hand without touching the sketch
first.

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
5. After boot homing (claw starts fully open), send `G` - should spin
   closed and stop on its own, grabbing the pen.
6. Send `O` - should spin open (overshooting) and stop on its own, popping
   the pen loose.
7. Send `R` - should spin closed briefly and stop, back at the fully-open
   position `G` expects, ready to send `G` again.
8. Once you've found a speed/direction you're happy with for open/close, use
   `A<value>` to explore further from 90 and confirm those values spin
   cleanly (not straining/stalling) before relying on `O`/`G`/`R`.

To re-measure `kTimedMoveDurationMs` by hand if the gripper mechanism
changes: send `C`, count seconds, send `S` right when it reaches the
position you want, note the time, and update the constant (and the
per-command fractions in `HandleByte()`) in the sketch.

Also test once with the USB cable unplugged from the computer (ESP32 powered
some other way, e.g. a battery/second USB source) to confirm the firmware
doesn't have a hidden dependency on USB 5V for the logic side - only the
servo's own separate supply should matter for the servo itself.

### Testing RS485

Once the transceiver is wired (see "RS485" above), connect a second
USB-RS485 adapter to a computer, wire its A/B to the transceiver's A/B (with
the ESP32 still separately powered), and send the same commands from a
serial terminal at the RS485 baud rate (115200 by default) instead of the
Serial Monitor. Confirm the gripper responds identically to USB, and that a
`value=<n>` line comes back over RS485 after each command. Then unplug USB
entirely (leave the ESP32 powered some other way) and repeat, to confirm
RS485-only operation - the state it'll actually run in once wired to the
robot controller.

## Not in scope here

No ROS2/micro-ROS integration, no changes to `ros2/robross_painter` or
`docs/painting-paths-format.md`. Those are separate follow-up work once
there's a decision on where the gripper physically mounts and whether it
reuses the `lower_tool`/`lift_tool` command semantics or gets its own new
command type.
