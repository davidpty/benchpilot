# ESP32 Implementation

## 1. Overview

This document describes the BenchPilot implementation of SeaTalk 1 communication on an ESP32 using a discrete transistor level shifter (BC557/BC547) circuit. The ESP32 (ESP32-2432S028 CYD) replaces the Arduino Pro Micro used in reference projects.

## 2. Hardware Interface

### BenchPilot Circuit Board

The BenchPilot level shifter uses two discrete transistors (no 74LS07):

| Component | Function |
|-----------|----------|
| Q1 (BC557 PNP) | Inverting level translator: 12V SeaTalk → 3.3V ESP32 RX |
| Q2 (BC547 NPN) | Inverting level translator: 3.3V ESP32 TX → 12V SeaTalk |
| R1 (10K), R2 (2.2K) | Base bias for Q1 |
| R3 (10K), R4 (3.9K) | Base pulldown + current limit for Q2 |
| R5 (6.8K), R6 (27K) | Voltage divider on Q1 output for ESP32 safe level (~2.5V) |
| U1 (DC-DC) | 12V → 3.3V step-down for ESP32 power |

### Circuit Behavior

| Path | ESP32 Pin | Transistor | SeaTalk Bus |
|------|-----------|------------|-------------|
| TX idle | GPIO 22 = 0V (LOW) | Q2 OFF | 12V (idle, correct) |
| TX active | GPIO 22 = 3.3V (HIGH) | Q2 ON | 0V (active, correct) |
| RX idle | GPIO 35 = 0V (LOW) | Q1 OFF | 12V (idle, correct) |
| RX active | GPIO 35 = ~2.5V (HIGH) | Q1 ON | 0V (active, correct) |

Both paths are **inverting**. The firmware compensates via `uart_set_line_inverse()`.

### GPIO Pin Mapping (Actual)

| Signal | ESP32 Pin | Notes |
|--------|-----------|-------|
| SeaTalk RX | GPIO 35 | UART2 RX, inverted via Q1 |
| SeaTalk TX | GPIO 22 | UART2 TX, inverted via Q2 |
| TFT DC | GPIO 2 | CYD display — do not use for other purposes |
| TFT Backlight | GPIO 21 | Controlled via TFT_BL_PIN |
| Display | ILI9341 | SPI, 40 MHz, rotation 1 (landscape) |

### Power

- ESP32 powered via on-board DC-DC step-down from 12V SeaTalk bus
- The CYD board has its own 3.3V regulator — no LM7805 needed
- Current budget: ESP32 ~80mA, display backlight ~40mA, interface ~5mA, total <150mA

## 3. UART Configuration

### Current Approach (8N1 with err_wr_mask = 0)

The firmware uses UART2 in standard 8N1 mode at 4800 baud. SeaTalk requires 9-bit (8 data + 1 command bit), but the 9th bit (command marker) is **not distinguished** in the current receive-only implementation — all bytes are treated as data. This works because:

1. SeaTalk command bytes always have the correct framing from the bus
2. The UART will frame-sync on the stop bit of each byte
3. Datagram boundaries are determined by the message length field (byte 1 low nibble), not the 9th bit

### Register Setup

```cpp
Seatalk.begin(4800, SERIAL_8N1, SEATALK_RX_PIN, SEATALK_TX_PIN);

// Both RX and TX are inverted by the transistor circuit
uart_set_line_inverse(UART_NUM_2,
    (SEATALK_RX_INVERT ? UART_SIGNAL_RXD_INV : 0) |
    (SEATALK_TX_INVERT ? UART_SIGNAL_TXD_INV : 0));

// Clear UART error-write-mask to avoid dropped bytes on framing errors
REG_CLR_BIT(UART_CONF0_REG(UART_NUM_2), UART_ERR_WR_MASK);
```

The `REG_CLR_BIT` macro (from `soc/uart_reg.h`) is used instead of struct-based register writes to avoid racing with the UART ISR (which caused boot loops).

### Future: 9-Bit Mode

For full 9-bit support (needed for transmit with command-bit awareness), options include:

1. **UART RS485 mode** with 9-bit addressing — use `UART_RS485_CONF_REG`
2. **RMT peripheral** — bit-bang with precise hardware timing
3. **9-bit register hack** — `WRITE_PERI_REG(UART_CONF0_REG(2), ...)` to set `UART_BIT_NUM = 3`

The current implementation does not require 9-bit mode for receive.

## 4. Receive Strategy

### UART Polling in loop()

```cpp
while (Seatalk.available() > 0) {
    uint8_t b = Seatalk.read();
    totalBytes++;
    lastPacketTime = millis();
    processByte(b);
}
```

No interrupts. The main loop polls UART2 at every iteration. At 4800 baud (~2ms per byte), this is well within the ESP32's capacity.

### Datagram Framing (processByte)

The `processByte()` function implements a simple byte-oriented state machine:

1. Buffer resets after a 50ms inter-message timeout (gap between SeaTalk datagrams)
2. On receiving ≥3 bytes, reads the length from byte 1's low nibble: `len = 3 + (attr & 0x0F)`
3. When the full datagram is assembled, parses known message types (0x84, 0x9C, 0x89, 0x10)
4. Message text is logged to a circular buffer for TFT display and web UI

### SeaTalk Message Decoding

| Command | Description | Fields Extracted |
|---------|-------------|-----------------|
| 0x84 (9 bytes) | Autopilot status | Heading, rudder position |
| 0x9C (4 bytes) | Compass heading + rudder | Heading, rudder |
| 0x89 (5 bytes) | Steering compass course | Heading |
| 0x10 (4 bytes) | Apparent wind angle | Heading (from wind instrument) |
| 0x90 (5 bytes) | Device identification | System type |

Heading decode formula (used for 0x84, 0x9C, 0x89, 0x10):

```cpp
uint8_t u = (byte1 >> 4) & 0x0F;
int deg = ((u & 3) * 90) + ((byte2 & 0x3F) * 2)
        + ((u & 0x0C) ? (((u & 0x0C) == 0x0C) ? 2 : 1) : 0);
if (deg > 360) deg -= 360;
```

### Logging

- Ring buffer: 30 entries, 19 shown on TFT, 20 shown on web UI
- Each entry: hex dump of the datagram bytes + decoded heading/rudder if applicable

## 5. Transmit Strategy (Planned)

### TX Pin Configuration

TX is configured on GPIO 22 with **inversion enabled** (`UART_SIGNAL_TXD_INV`). The UART idle state (logic 1) becomes 0V on the pin, keeping Q2 OFF and the bus at 12V. When a byte is transmitted:

1. Start bit (logic 0) → inverted to 3.3V → Q2 ON → bus pulled to 0V
2. Data bits are transmitted LSB-first, each inverted by the UART
3. Stop bit (logic 1) → inverted to 0V → Q2 OFF → bus returns to 12V

### CSMA/CD Implementation (from AK-Homberger pattern)

When transmit is implemented, the algorithm will be:

```cpp
bool sendDatagram(uint8_t *data, int len) {
    for (int retry = 0; retry < 5; retry++) {
        // 1. Listen — drain any pending RX bytes
        while (Seatalk.available()) {
            Seatalk.read();
            delay(3);
        }
        // 2. Transmit byte-by-byte with collision check
        bool ok = true;
        for (int i = 0; i < len; i++) {
            Seatalk.write(data[i]);
            delay(3);
            if (Seatalk.available()) {
                uint8_t inbyte = Seatalk.read();
                if (inbyte != data[i]) { ok = false; break; }
            } else { ok = false; break; }
        }
        if (ok) return true;
        delay(random(2, 50));  // random backoff
    }
    return false;
}
```

### Command Queue

- Single command buffer (no queue depth needed for user-initiated commands)
- 300ms debounce between commands (ST6002 limit)
- Commands: 0x86 autopilot keystrokes (Auto, Standby, +1°, -1°, +10°, -10°)

## 6. Software Architecture

### Module Layout (Actual)

```
src/
├── main.cpp                 # Main firmware: WiFi, OTA, WebServer, SeaTalk RX
├── wifi_config.h            # WiFi credentials (gitignored)
lib/cyd_config/
├── User_Setup.h             # TFT_eSPI configuration for CYD
docs/
├── seatalk_protocol.md      # SeaTalk protocol reference
├── seatalk_messages.md       # Message dictionary
├── autopilot_commands.md     # Autopilot control reference
├── esp32_implementation.md   # This document
├── SeaTalkToESP32-Circuit.txt # Circuit schematic description
├── SeaTalkToESP32-Circuit.png # Circuit schematic image
```

### Firmware Architecture (main.cpp)

```
setup()
├── Backlight on (GPIO 21)
├── TFT init (ILI9341, rotation 1)
├── drawStatusBar()
├── WiFi begin (STA mode)
├── ArduinoOTA begin
├── UART2 begin (4800 8N1, RX=35, TX=22)
│   ├── uart_set_line_inverse (RX + TX)
│   └── REG_CLR_BIT (err_wr_mask)
└── WebServer begin

loop()
├── ArduinoOTA.handle()
├── server.handleClient()
├── WiFi status check
├── while Seatalk.available():
│   ├── read byte
│   ├── totalBytes++
│   └── processByte()
└── updateDisplay() (every 500ms)
```

### Configurable Parameters

| Parameter | Value | Defined In |
|-----------|-------|------------|
| Baud rate | 4800 | `SEATALK_BAUD` |
| RX pin | GPIO 35 | `SEATALK_RX_PIN` |
| TX pin | GPIO 22 | `SEATALK_TX_PIN` |
| RX invert | ON | `SEATALK_RX_INVERT` |
| TX invert | ON | `SEATALK_TX_INVERT` |
| Blue LED pin | GPIO 27 | `TFT_BL_PIN` |
| Message log size | 30 entries | `MSG_LOG_SIZE` |
| Display refresh | 500ms | Hard-coded in `updateDisplay()` |
| Bus idle timeout | 50ms | Hard-coded in `processByte()` |

## 7. Differences from Reference Implementations

| Aspect | AK-Homberger (Arduino) | James Roscoe (Linux) | BenchPilot (ESP32) |
|--------|----------------------|---------------------|---------------------|
| MCU | ATmega32U4 @ 16 MHz | Linux (RPi GPIO) | ESP32 @ 240 MHz |
| Level shifter | 74LS07 (non-inverting) | None (3.3V GPIO direct) | BC557/BC547 (inverting ×2) |
| TX compensation | None needed | None (bit-bang) | `UART_SIGNAL_TXD_INV` |
| RX compensation | None needed | Inverted GPIO reads | `UART_SIGNAL_RXD_INV` |
| 9-bit mode | Patched HardwareSerial | Bit-bang with hrtimer | err_wr_mask = 0 (8N1) |
| Display | OLED SSD1306 (I2C) | None | TFT ILI9341 (SPI) |
| Communication | 433 MHz RF + serial | OpenCPN plugin | WiFi + HTTP API |
| Bus arbitration | CSMA/CD with UART | CSMA/CD with bit-bang | Not yet implemented |

## 8. Key Source Files Reference

| File | Location | What to Steal |
|------|----------|---------------|
| `ArduinoPilotMicro433WindNG.ino` | `refs/Seatalk-Autopilot-Remote-Control/` | CSMA/CD sendDatagram(), message constants |
| `seatalk_datagram.c` | `refs/seatalk/` | All build/parse functions, compass encoding |
| `seatalk_datagram.h` | `refs/seatalk/` | All message constants, command enums |
| `seatalk_transport_layer.c` | `refs/seatalk/` | State machine, byte Tx/Rx logic |
| `autopilot_pi.cpp` | `refs/raymarine_autopilot_pi/` | 0x84/0x86/0x92 complete reference |
| `HardwareSerial9bit/` | `refs/Seatalk-Autopilot-Remote-Control/` | 9-bit serial pattern for reference |
