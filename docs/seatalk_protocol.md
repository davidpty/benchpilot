# SeaTalk 1 Protocol Reference

## 1. Physical Layer

SeaTalk 1 is a single-wire bus running at 12V nominal with open-collector signaling.

| Property | Value |
|---|---|
| Bus voltage | 12V nominal, pulled high via 10K resistor |
| Logic 1 (idle) | Bus line high (12V) |
| Logic 0 (active) | Bus line driven low (GND) |
| Drive type | Open-collector (wired-AND) |
| Cable | 3-wire: +12V, Signal (ST), GND |

The bus is **inverted** relative to typical UART. Idle state is high (12V). Start bit is a high-to-low transition.

### Interface Circuit (BenchPilot discrete transistor design)

The BenchPilot board uses two discrete transistors (BC557 PNP, BC547 NPN) as an inverting level shifter between 12V SeaTalk and 3.3V ESP32 GPIO:

- **Transmit path**: ESP32 TX (GPIO 22) → R4 (3.9K) → Q2 (BC547 NPN) base. Q2 collector tied to SeaTalk DATA, emitter to GND. R3 (10K) pull-down on TX line keeps Q2 off during ESP32 boot.
- **Receive path**: SeaTalk DATA → R2 (2.2K) to 12V, R1 (10K) to Q1 (BC557 PNP) base. Q1 emitter tied to 12V. Q1 collector → R6 (27K) → ESP32 RX (GPIO 35) with R5 (6.8K) pull-down to GND.
- **Pull-up**: 10K resistor from SeaTalk line to +12V (on the Raymarine bus backbone).

Signal flow:

| SeaTalk bus | Receive (Q1) | ESP32 RX | Transmit (Q2) | SeaTalk bus |
|-------------|--------------|----------|---------------|-------------|
| 12V (idle)  | OFF          | 0V (LOW)  | ESP32 TX 0V   | OFF → 12V   |
| 0V (active) | ON           | ~2.5V (HIGH) | ESP32 TX 3.3V | ON → 0V    |

**Both RX and TX are inverting paths.** The firmware must compensate:

| Path | Inversion | Compensation |
|------|-----------|-------------|
| RX   | Q1 PNP inverts (12V→LOW, 0V→HIGH) | `uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV)` |
| TX   | Q2 NPN inverts (0V→OFF→12V, 3.3V→ON→0V) | `uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV)` |

See `docs/SeaTalkToESP32-Circuit.txt` or `docs/SeaTalkToESP32-Circuit.png` for the full schematic.

### Interface Circuit (Homberger design — reference only, not used here)

The AK-Homberger circuit uses a 74LS07 open-collector hex buffer for level translation between the 12V SeaTalk bus and 5V Arduino GPIO:

- **Transmit path**: Arduino TX → 74LS07 buffer input → open-collector output with 68K series resistor (R2) to SeaTalk line. When low, pulls bus to GND.
- **Receive path**: SeaTalk line via 68K (R2) → 74LS07 buffer input → output with 10K pull-up (R5) to Arduino RX. 5.1V Zener (D1) clamps voltage. 27K pulldown (R3) provides bleed path.
- **Pull-up**: 10K resistor (R1) from SeaTalk line to +12V.

Key difference: The 74LS07 is **non-inverting**, so only RX needs firmware inversion (the UART idle-HIGH maps through correctly for TX). Our discrete transistor circuit inverts both paths.

## 2. Serial Format

| Parameter | Value |
|---|---|
| Baud rate | **4800** baud |
| Data bits | **8** (LSB first) |
| Extra bit | **9th bit** — command/address marker |
| Parity | None |
| Stop bits | 1 |
| Byte format | `start + 8 data + command + stop` |

### Bit Timing

| Period | Duration | Notes |
|---|---|---|
| Bit interval | 208,333 ns (208.3 μs) | 1/4800 s |
| Start bit sample delay | 52,083 ns (52 μs) | 1/4 bit period after falling edge |
| Full byte time | ~2.29 ms | 11 bit periods |
| Inter-byte guard | 10 bit periods (~2.08 ms) | After transmission, bus waits idle |

### 9th Bit (Command Bit)

The 9th bit is the defining feature of SeaTalk:

- **1** (high) = first byte of a new datagram (command/address byte)
- **0** (low) = subsequent bytes within the same datagram (data byte)

The 9th bit is NOT parity. It functions as a datagram frame marker, allowing receivers to resynchronize on the bus.

AK-Homberger stores bytes as `uint16_t` with bit 8 (0x100) set for command bytes:

```
0x186 = binary 1 1000 0110
                ^--- 9th bit = 1 (command byte)
                    ^^^^^^^^--- lower 8 bits = 0x86
```

### Start Bit Detection

A falling edge (high→low transition) on the idle bus signals a start bit. The receiver:
1. Detects falling edge via GPIO interrupt
2. Waits 52 μs (1/4 bit period) before sampling first data bit
3. Samples subsequent bits every 208.3 μs
4. After 8 data bits + 1 command bit, checks stop bit (must be high)

If stop bit is low, the byte is discarded (misalignment recovery strategy).

### Stop Bit Validation

From `seatalk_transport_layer.c:138-145`:
```c
if (!bit_value) {
    // stop bit needs to be high. If not, we are misaligned
    // with the actual bytes and must discard this byte.
    // Eventually "walk" around to correct start/stop alignment.
    set_bus_state(BUS_STATE_IDLE);
    return 0;
}
```

## 3. Bus Arbitration (CSMA/CD)

SeaTalk uses a Carrier Sense Multiple Access with Collision Detection (CSMA/CD) protocol:

### Protocol

1. **Listen before talk**: Before transmitting, drain all incoming bytes from UART buffer (wait for bus silence).
2. **Transmit byte-by-byte**: Send each byte, then immediately read back from the bus.
3. **Collision detection**: If the received byte does not match the transmitted byte, a collision occurred.
4. **Backoff**: Wait a random interval (2-50 ms), then retry up to 5 times.

### AK-Homberger Implementation (`sendDatagram`)

```
for each of 5 retry attempts:
    drain UART RX buffer
    for each byte in datagram:
        write byte to UART
        delay 3ms
        read back from UART
        if received != sent → collision, break
    if all bytes OK → return success
    delay random(2, 50ms)
return failure
```

### James Roscoe Implementation (Linux)

Uses a bus state machine with four states:

```
BUS_STATE_IDLE → BUS_STATE_RECEIVING → BUS_STATE_WAIT_FOR_IDLE
              → BUS_STATE_TRANSMITTING
```

- During transmission, each byte is compared against what appears on the bus
- If mismatch: `cancel_datagram()` — drains remaining bytes, returns to IDLE
- After transmitting, bus idles for 10 bit periods (WAIT_FOR_IDLE) before next transmission
- High-priority commands can interrupt lower-priority sensor transmissions

### Collision Flow

```
Transmitter sends byte
    ↓
Transmitter reads byte back from bus
    ↓
if (read_byte != sent_byte):
    cancel_datagram()     ← collision detected
    wait
    retry from start
```

## 4. Packet / Datagram Structure

### General Format

```
+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 |  ...   | Byte N
| CMD    | LEN    | DATA_0 | DATA_1 |  ...   | DATA_N-2
+--------+--------+--------+--------+--------+
 9th=1     9th=0    9th=0    9th=0           9th=0
```

| Offset | Field | Description |
|---|---|---|
| 0 | Command | Message type identifier. 9th bit = 1 |
| 1 | Length+Flags | `[High nibble: flags/qualifiers][Low nibble: payload length]` |
| 2..N-1 | Payload | Message-specific data bytes. N = (low nibble of byte 1) + 3 |
| N | Checksum | Optional. Present only in specific message types (0x82, 0x85, 0x86) |

### Length Encoding

The low nibble of byte 1 encodes the payload length:

```
total_datagram_bytes = (byte1 & 0x0F) + 3
payload_bytes = total_datagram_bytes - 3  (= low nibble of byte 1)
```

Example: `0x21` → low nibble = 1 → total = 4 bytes, payload = 1 byte
Example: `0x26` → low nibble = 6 → total = 9 bytes, payload = 6 bytes

### Second Byte High Nibble

The high nibble often carries additional flags or qualifiers specific to the message type. For example:

- `build_autopilot_status()`: high nibble of byte 1 encodes compass heading bits
- `build_lamp_intensity()`: high nibble of byte 1 encodes intensity level
- `build_autopilot_command()`: high nibble identifies remote type (1=Z101, 2=ST600R)

### Initialization Pattern

From `seatalk_datagram.c:113-118`:
```c
void initialize_datagram(char *datagram, int datagram_number,
                         int total_length, int high_nibble_of_second_byte) {
    datagram[0] = datagram_number;
    datagram[1] = (total_length - 3) + (high_nibble_of_second_byte << 4);
    for (int i = 2; i < total_length; i++) { datagram[i] = 0; }
}
```

## 5. Checksum Calculation

SeaTalk uses different checksum schemes depending on the message type:

### A. Complement Checksum (0x86, 0x85, 0x82)

For messages with command byte 0x86, 0x85, and 0x82, the checksum is the **bitwise complement** of the associated data byte(s):

```c
int complement_checksum(int value) {
    return value ^ 0xff;  // bitwise NOT
}
```

**Command 0x86** (autopilot keystroke): `86 21 <cmd> <cmd ^ 0xFF>`
- `0x86, 0x21, 0x01, 0xFE` — Auto command (0x01 ^ 0xFF = 0xFE)
- `0x86, 0x21, 0x07, 0xF8` — +1 degree (0x07 ^ 0xFF = 0xF8)

**Command 0x85** (navigation to waypoint): `85 x6 ... <yf> 00 <yf ^ 0xFF>`
- Checksum covers byte 6, stored in byte 8
- Validated as `(yf | yf_comp) == 0xFF`

**Command 0x82** (target waypoint name): Each 6-bit character is stored with its complement in the next byte:
```
xx !xx yy !yy zz !zz
```
Validated as `(xx + xx_comp) == 0xFF`

### B. No Checksum (most messages)

Most SeaTalk messages (0x00, 0x10, 0x11, 0x20, 0x84, 0x89, 0x8C, etc.) do NOT have a checksum byte. The message is validated by:
1. Correct length (from byte 1 low nibble)
2. Correct 9th bit framing
3. Valid stop bit

### C. Checksum Summary

| Command | Checksum type | Validation |
|---|---|---|
| 0x82 | Complement per byte | `(xx + !xx) == 0xFF` |
| 0x85 | Complement of byte 6 in byte 8 | `(yf | yf_comp) == 0xFF` |
| 0x86 | Complement of byte 2 in byte 3 | `(yy | yy_comp) == 0xFF` |
| All others | No checksum byte | Length + framing only |

## 6. Receive Pipeline (James Roscoe Linux Architecture)

```
GPIO pin interrupt (falling edge)
    ↓
initiate_seatalk_receive_character()
    ↓  (start bit timer)
seatalk_receive_bit()  ← called at 208.3 μs intervals
    ↓
seatalk_byte_received(byte, command_bit)
    ↓
handle_seatalk_datagram(datagram)
    ↓
update_seatalk_state(datagram) ← giant switch on byte 0
```

### State machine per received byte

```
command_bit == 1:
    reset receive buffer (start new datagram)
    store byte 0 (command)

command_bit == 0 (first data byte = byte 1):
    store byte 1
    extract length from low nibble
    set remaining byte counter

command_bit == 0 (subsequent bytes):
    store byte
    decrement remaining counter
    if counter == 0: dispatch datagram to handler
```

## 7. Transmit Pipeline

```
Command or sensor data ready
    ↓
wake_transmitter() → initiate_seatalk_transmitter()
    ↓
seatalk_transmit_bit()  ← called at 208.3 μs intervals
    ↓
  bit -1: check can_transmit(), get next byte, set start bit (low)
  bit 0-7: send data bits LSB first
  bit 8: send command bit (1 for first byte, 0 otherwise)
  bit 9: send stop bit (high), signal byte complete
```

After byte completion, the transmitter checks for more pending bytes. Between datagrams, the bus idles for 10 bit periods.

## 8. NMEA0183 Wrapping (James Roscoe Plugin)

For SeaTalk-over-NMEA0183 (SeaTalk Link converters), messages are wrapped in NMEA sentences:

```
$STALK,<hex_bytes_comma_separated>*<XOR_checksum>
$STALK,84,26,A2,88,40,00,FE,02,06*15
$STALK,86,21,01,FE*4E
```

The checksum is XOR of all bytes between `$` (exclusive) and `*` (exclusive).

## 9. Key References

| Source | Focus | Language |
|---|---|---|
| Thomas Knauf (thomasknauf.de/seatalk.htm) | Original protocol reference | German/English |
| James Roscoe (seatalk library, refs/seatalk/) | Complete C library implementation | C (Linux + µC) |
| AK-Homberger (refs/Seatalk-Autopilot-Remote-Control/) | Arduino autopilot remote | C++ (Arduino) |
| James Roscoe (raymarine_autopilot_pi) | OpenCPN autopilot plugin | C++ (wxWidgets) |
| `docs/SeaTalkToESP32-Circuit.txt` | BenchPilot discrete transistor circuit | — |
| `src/main.cpp` | BenchPilot firmware | C++ |
