// RS485 wiring self-test - NOT the real gripper firmware (see
// ../gripper_esp32/ for that). Tests only the RS485 link itself, in both
// directions, independent of the servo/gripper protocol - so a wiring
// mistake is easy to isolate before layering the real gripper commands
// on top. See ../gripper_esp32/README.md "RS485" section for the full
// wiring table this assumes:
//   MAX485 TXD -> GPIO16, RXD -> GPIO17, DE+RE (tied together) -> GPIO21,
//   GND common with the ESP32, A/B -> your USB-RS485 adapter's T/R+/T/R-.
//
// What it does:
//   - Every second, transmits a heartbeat line out over RS485
//     ("heartbeat #<n>"). If these show up in a terminal (CoolTerm, screen,
//     etc.) connected through the USB-RS485 adapter, the ESP32 -> PC
//     direction of the link works (MAX485 TXD -> GPIO16 wiring, DE/RE,
//     baud rate, and A/B polarity are all correct).
//   - Echoes back anything it receives over RS485, prefixed "echo: ".
//     Type characters in the same terminal; if they come back echoed, the
//     PC -> ESP32 direction also works (MAX485 RXD -> GPIO17 wiring).
//   - Mirrors both directions to USB Serial (115200) too, so if the ESP32
//     is still connected over USB at the same time you can cross-check
//     locally without needing the RS485 terminal.
//
// Reading the result:
//   - Heartbeats show up AND echo works -> both directions of the RS485
//     link are wired correctly; the real gripper firmware should work
//     over RS485 too.
//   - No heartbeat at all -> check GND common, VCC/logic-level match,
//     baud rate (115200), and that the terminal is pointed at the
//     USB-RS485 adapter's port, not some other device.
//   - Heartbeat shows up but echo doesn't -> ESP32 -> PC direction is
//     fine; PC -> ESP32 direction is broken - check MAX485 RXD -> GPIO17
//     and that DE/RE are both tied to GPIO21 (not just one of them).
//   - Echo works but no heartbeat (unlikely, since echo already proves
//     both directions) -> restart the sketch and re-check timing; not a
//     wiring symptom.

const int kRs485RxPin = 16;
const int kRs485TxPin = 17;
const int kRs485DePin = 21;
const unsigned long kRs485Baud = 115200;
const unsigned long kHeartbeatIntervalMs = 1000;

unsigned long last_heartbeat_ms = 0;
unsigned long heartbeat_count = 0;

// Same half-duplex direction handling as the real gripper firmware: HIGH
// enables the transceiver's driver (transmit), LOW releases the bus back
// to receive. The short delay lets the transceiver chip actually switch.
void Rs485SetTransmit(bool enable) {
  digitalWrite(kRs485DePin, enable ? HIGH : LOW);
  delayMicroseconds(10);
}

void Rs485Print(const String &msg) {
  Rs485SetTransmit(true);
  Serial2.print(msg);
  Serial2.flush();
  Rs485SetTransmit(false);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(kRs485Baud, SERIAL_8N1, kRs485RxPin, kRs485TxPin);
  pinMode(kRs485DePin, OUTPUT);
  Rs485SetTransmit(false);  // start in receive mode

  Serial.println("RS485 self-test starting.");
  Serial.println("- Sending a heartbeat over RS485 every second.");
  Serial.println("- Anything received over RS485 gets echoed back, prefixed 'echo: '.");
  delay(200);
  Rs485Print("RS485 self-test ready\n");
}

void loop() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    Serial.print("rx: ");
    Serial.println(c);

    String reply = "echo: ";
    reply += c;
    reply += "\n";
    Rs485Print(reply);
  }

  if (millis() - last_heartbeat_ms >= kHeartbeatIntervalMs) {
    last_heartbeat_ms = millis();
    heartbeat_count++;
    String msg = "heartbeat #";
    msg += heartbeat_count;
    msg += "\n";
    Rs485Print(msg);
    Serial.print("tx: ");
    Serial.print(msg);
  }
}
