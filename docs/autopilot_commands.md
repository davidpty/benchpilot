# Autopilot Control Reference

## System Architecture

```
ESP32 + BenchPilot Level Shifter (BC557/BC547)
        |
        |  SeaTalk 1 bus (4800 baud, 9-bit, 12V open-collector)
        |
Raymarine ST6002 Display (control head)
        |
Raymarine SmartPilot S3 Course Computer
        |
        |  Motor drive unit
        |
Rudder
```

The ST6002 is the display/control head on the bus. The SmartPilot S3 is the course computer that drives the rudder. The ESP32 (via the discrete transistor level shifter) appears as another device on the bus and can both send commands to and receive status from the autopilot.

## Key Messages for Autopilot

### Transmitted by ESP32 (to control autopilot)

| Message | Command | Function |
|---------|---------|----------|
| 0x86 | Autopilot keystroke | Mode changes, heading changes |
| 0x92 | Set parameter | Response, wind trim, rudder gain |
| 0x87 | Set response level | Deadband control |

### Received by ESP32 (from autopilot)

| Message | Command | Function |
|---------|---------|----------|
| 0x84 | Autopilot status | Heading, mode, rudder, alarms |
| 0x89 | Compass course | Heading from steering compass |
| 0x9C | Heading + rudder | Combined heading and rudder |
| 0x88 | Parameter value | Current parameter settings |
| 0x91 | Rudder gain | Current gain setting |

## Autopilot Commands (0x86)

### Remote Type Encoding

The high nibble of byte 1 encodes the remote type:
- `0x1` = Z101 remote (only sends course changes in Auto mode)
- `0x2` = ST600R remote (full key set — preferred for ESP32)

The AK-Homberger code hard-codes `x = 2` (ST600R remote).

### Command Formats

| Function | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Hex |
|----------|--------|--------|--------|--------|-----|
| Auto | 0x86 | 0x21 | 0x01 | 0xFE | `86 21 01 FE` |
| Standby | 0x86 | 0x21 | 0x02 | 0xFD | `86 21 02 FD` |
| Track | 0x86 | 0x21 | 0x03 | 0xFC | `86 21 03 FC` |
| Cycle Display | 0x86 | 0x21 | 0x04 | 0xFB | `86 21 04 FB` |
| -1 degree | 0x86 | 0x21 | 0x05 | 0xFA | `86 21 05 FA` |
| -10 degrees | 0x86 | 0x21 | 0x06 | 0xF9 | `86 21 06 F9` |
| +1 degree | 0x86 | 0x21 | 0x07 | 0xF8 | `86 21 07 F8` |
| +10 degrees | 0x86 | 0x21 | 0x08 | 0xF7 | `86 21 08 F7` |
| Decrease Gain | 0x86 | 0x21 | 0x09 | 0xF6 | `86 21 09 F6` |
| Increase Gain | 0x86 | 0x21 | 0x0A | 0xF5 | `86 21 0A F5` |
| Wind/Vane mode | 0x86 | 0x21 | 0x23 | 0xDC | `86 21 23 DC` |
| Return to course | 0x86 | 0x21 | 0x41 | 0xBE | `86 21 41 BE` |

### Extended Commands (Long Press / Hold)

When a key is held for >1 second, the command byte has 0x40 OR'd in:

| Function | Command | Notes |
|----------|---------|-------|
| -1 held | 0x45 | Repeated while held |
| -10 held | 0x46 | Repeated while held |
| +1 held | 0x47 | Repeated while held |
| +10 held | 0x48 | Repeated while held |
| Track held | 0x43 | Long press |
| Disp held | 0x44 | Long press |
| Auto held | 0x41 | Also = Return to course |

When a key is held in the "repeat" pattern (0x80-0x83):
| Function | Command | Notes |
|----------|---------|-------|
| Hold -1 | 0x80 | Repeated every second |
| Hold -10 | 0x81 | Repeated every second |
| Hold +1 | 0x82 | Repeated every second |
| Hold +10 | 0x83 | Repeated every second |
| Release held key | 0x84 | Sent when held key is released |

## Messages Received from Autopilot

### 0x84 Status — Complete Field Reference

```
84 u6 vw xy 0z 0m rr ss tt
```

#### Heading Decoding

The compass heading is encoded across byte 1 (high nibble `u`) and byte 2 (`vw`):

```
u = first_nibble(datagram[1])    // from high nibble of byte 1
vw = datagram[2]                  // byte 2
xy = datagram[3]                  // byte 3
z = last_nibble(datagram[4])      // byte 4 low nibble
m = last_nibble(datagram[5])      // byte 5 low nibble
rr = datagram[6]                  // byte 6 (signed)
ss = datagram[7]                  // byte 7
tt = datagram[8]                  // byte 8
```

**Compass heading:**
```c
heading = ((u & 0x03) * 90) + ((vw & 0x3f) * 2) + offset
where offset = 0 if (u & 0x0c) == 0
               1 if (u & 0x0c) == 0x04
               1 if (u & 0x0c) == 0x08
               2 if (u & 0x0c) == 0x0c
// Simplified: offset = (u >> 3)  // bit 3 is 1 = +1°, bits 2-3 both 1 = +2°
```

**Target heading:**
```c
target = (((vw & 0xc0) >> 6) * 90) + (xy / 2)
```

#### Example: 84 26 A2 88 40 00 FE 02 06

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | 0x84 | Autopilot status |
| 1 | 0x26 | u = 2, length = 6 (9 bytes total) |
| 2 | 0xA2 | vw = 162 |
| 3 | 0x88 | xy = 136 |
| 4 | 0x40 | z = 0 (Standby) |
| 5 | 0x00 | m = 0 (no alarms) |
| 6 | 0xFE | rr = -2° (rudder 2° left) |
| 7 | 0x02 | ss = 0x02 (display on, 400G) |
| 8 | 0x06 | tt = 0x06 (computer type) |

**Decoded:**
- Heading: `((2 & 3) * 90) + ((162 & 0x3f) * 2) + ((2 & 0x0c) ? (((2 & 0x0c) == 0x0c) ? 2 : 1) : 0)`
  = `(2 * 90) + (34 * 2) + (0)` = 180 + 68 = **248°**
- Target: `(((162 & 0xc0) >> 6) * 90) + (136 / 2)` = `(2 * 90) + 68` = **248°**
- Mode: Standby
- Rudder: -2° (FE = -2 in two's complement)

#### Mode State Table

| z value | AUTOPILOT_MODE | Description |
|---------|----------------|-------------|
| 0x00 | STANDBY | Autopilot disengaged; manual steering |
| 0x02 | AUTO | Autopilot engaged, holding heading |
| 0x04 | VANE (WIND) | Autopilot tracking wind angle |
| 0x08 | TRACK | Autopilot navigating to waypoint |

#### Alarm / Sub-mode

| m bit | Alarm | Description |
|-------|-------|-------------|
| 0x04 | Off Course | Boat deviated from target heading |
| 0x08 | Wind Shift | Wind angle changed significantly in Vane mode |

#### Display Flags

| ss bit | Meaning |
|--------|---------|
| 0x01 | Turn off heading display |
| 0x02 | Always on |
| 0x08 | "No data" |
| 0x10 | "Large XTE" (large cross-track error) |
| 0x80 | "Auto rel" (auto release) |

## Parameter Commands

### 0x92 — Set Parameter (5 bytes)

**Format**: `92 02 <param> <value> 00`

| Parameter ID | Name | Range | Description |
|-------------|------|-------|-------------|
| 0x01 | Rudder Gain | 1-9 | Controls how aggressively the pilot responds to heading error |
| 0x11 | Wind Trim | 1-9 | Wind angle adjustment in Vane mode |
| 0x12 | Response Level | 1-9 | Overall autopilot response/aggressiveness |

### Reading Parameter Values

To trigger the autopilot to broadcast its current parameter settings, send the 0x92 command followed by a 0x86 display command. The autopilot responds with 0x88 or 0x91 on the bus.

From `raymarine_autopilot_pi`:
```
Get Response Level:  $STALK,92,02,12,0X,00
                     $STALK,86,21,2E,D1      (toggle response display)
Get Rudder Gain:     $STALK,92,02,01,0X,00
                     $STALK,86,21,6E,91      (enter rudder gain display)
Get Wind Trim:       $STALK,92,02,11,0X,00
```

Where `0X` is the desired value. The 0x86 display command causes the autopilot to echo the current parameter.

### 0x88 — Autopilot Parameter Broadcast (6 bytes) — Receive

**Format**: `88 03 ww xx yy zz`

| Byte | Field |
|------|-------|
| 2 | `ww` = parameter number |
| 3 | `xx` = current value |
| 4 | `yy` = minimum value |
| 5 | `zz` = maximum value |

### 0x91 — Set Rudder Gain / Response (3 bytes) — Both

**Format**: `91 00 0x`

When sent: set the rudder gain (1-9). When received: current rudder gain setting.

## Parameter 0x92 — Response Level (from raymarine_autopilot_pi plugin)

```
$STALK,92,02,12,<value>,00
```

Used to change the autopilot's response level (1-9, where 1 is least responsive, 9 is most responsive). The parameter is stored in the course computer EEPROM.

## Commands for ST6002 / SmartPilot S3

### Mode Transitions

```
                    ┌─────────┐
         ┌─────────→│ STANDBY │←──────────┐
         │          └─────────┘           │
         │  0x86/01    │                  │  0x86/02
         │             ▼                  │
         │          ┌──────┐              │
         ├─────────→│ AUTO │──────────────┤
         │          └──────┘              │
         │  0x86/23   │                   │
         │            ▼                   │
         │          ┌──────┐              │
         └─────────→│ WIND │──────────────┘
                     └──────┘
         ┌─────────→│ TRACK│──────────────┘
                     0x86/03
```

### Course Changes (in Auto or Wind mode)

When in AUTO or WIND mode, the +1/-1 and +10/-10 commands adjust the target heading:

```
+1 (0x07): Target heading += 1°
-1 (0x05): Target heading -= 1°
+10 (0x08): Target heading += 10°
-10 (0x06): Target heading -= 10°
```

The heading wraps around at 0/360.

### Implementation Guidance

**Timing:**
- Inter-command delay: at least 300 ms (AK-Homberger uses `KEY_DELAY=300` for debounce)
- Inter-byte delay: 3 ms between bytes in a single message
- Max retries: 5 attempts with random 2-50 ms backoff

**Keepalive:**
- NMEA Bridge ID (0x90): every 10 seconds (for SeaTalk-NG converter compatibility)
- Wind data refresh: monitor bus for 0x11 messages

**Safety:**
- Beep safety timeout: send beep-off after 3 seconds to prevent stuck alarm
- Data timeout: status values older than 5 seconds are considered stale
- Command timeout: 3 seconds max wait for command acknowledgement

### State Machine for ESP32

```
IDLE → waiting for commands (serial, RF, or network)
    ↓
COMMAND_RECEIVED → validate command
    ↓
CSMA_CD → listen for bus silence
    ↓
TRANSMIT → send datagram byte-by-byte with collision check
    ↓
ACK_WAIT → verify no collision
    ↓
IDLE → return to idle
```

On collision: random backoff (2-50ms), retry up to 5 times.
On persistent failure: report error, return to IDLE.

## NMEA0183 Wrapping (for reference via SeaTalk Link converters)

When the SeaTalk bus is accessed through an NMEA0183-to-SeaTalk converter (Raymarine SeaTalk Link or similar), messages are wrapped:

**Status received** (0x84 from autopilot):
```
$STALK,84,26,A2,88,40,00,FE,02,06*15
```

**Command sent** (0x86 to autopilot):
```
$STALK,86,21,01,FE*4E
$STALK,86,21,07,F8*4B
```

**Parameter request**:
```
$STALK,92,02,12,05,00*47
```

The NMEA XOR checksum is computed over all bytes between `$` (exclusive) and `*` (exclusive).

## Limitations and Safety Considerations

1. **Single master**: The SeaTalk bus is peer-to-peer; any device can transmit. But only one message at a time. Always use CSMA/CD.

2. **No guaranteed delivery**: There is no ACK/NAK protocol at the transport layer. Collision detection only verifies the bus state, not that the intended receiver processed the message.

3. **Mode-dependent commands**: Course change commands (+1/-1/+10/-10) are only valid when the autopilot is in AUTO, WIND, or TRACK mode. In STANDBY, they have no effect.

4. **Heading wrap**: 0-359 degrees. The autopilot handles shortest-path heading changes internally.

5. **Power cycling**: When the ST6002 or SmartPilot S3 is first powered, it broadcasts 0x83 (clear failure) and 0x89 (heading). Wait for a 0x84 status message before sending commands.

6. **Response level caution**: Setting response level too high (8-9) can cause unstable steering in rough seas. Start with level 3-5.

7. **Tacking**: The tack commands (0x21, 0x22) require the autopilot to be in WIND mode. They tack through the wind by ~90° (port or starboard).

8. **Held keys**: Holding a course change key (0x80-0x83) causes the heading to change continuously. Must send 0x84 to stop. Not recommended for ESP32 use unless specifically needed.

9. **Bus loading**: Keep message rate reasonable. The AK-Homberger project sends bridge ID every 10 seconds. Avoid flooding the bus.

10. **Data freshness**: Implement timeouts. If no 0x84 message received for 12 seconds (James Roscoe plugin watchdog), consider the autopilot offline.

## Relevant Source Files

| File | Repository | Content |
|------|-----------|---------|
| `seatalk_datagram.c:1030-1097` | James Roscoe (refs/seatalk) | Build/parse 0x84 status |
| `seatalk_datagram.c:1177-1211` | James Roscoe (refs/seatalk) | Build/parse 0x86 commands |
| `seatalk_datagram.h:13-45` | James Roscoe (refs/seatalk) | Command enum definitions |
| `seatalk_command.c:78-84` | James Roscoe (refs/seatalk) | Command send function |
| `seatalk_protocol.c:358-375` | James Roscoe (refs/seatalk) | 0x84 status update handler |
| `ArduinoPilotMicro433WindNG.ino` | AK-Homberger | Complete working implementation |
| `autopilot_pi.cpp` | raymarine_autopilot_pi | NMEA0183 SeaTalk wrapping |
