# SeaTalk 1 Message Dictionary

## Message Format Summary

All SeaTalk datagrams follow this structure:

```
Byte 0: Command byte (9th bit = 1)
Byte 1: [High nibble: flags] [Low nibble: payload_length = total_bytes - 3]
Byte 2..N-1: Payload (N = low_nibble + 3)
Byte N: Optional checksum (message-dependent)
```

## Complete Message Table

| Cmd | Len | Name | Direction | Source |
|-----|-----|------|-----------|--------|
| 0x00 | 5 | Depth Below Transducer | Receive | Knauf/Roscoe |
| 0x01 | — | Equipment ID | Receive | Knauf |
| 0x05 | 6 | Engine RPM and Blade Pitch | Receive | Knauf/Roscoe |
| 0x10 | 4 | Apparent Wind Angle | Both | Knauf/Roscoe |
| 0x11 | 4 | Apparent Wind Speed | Both | Knauf/Roscoe |
| 0x20 | 4 | Water Speed | Both | Knauf/Roscoe |
| 0x21 | 5 | Trip Mileage | Receive | Knauf/Roscoe |
| 0x22 | 5 | Total Mileage | Receive | Knauf/Roscoe |
| 0x23 | 4 | Water Temperature (basic) | Receive | Knauf/Roscoe |
| 0x24 | 5 | Display Units | Receive | Knauf/Roscoe |
| 0x25 | 7 | Total & Trip Log | Receive | Knauf/Roscoe |
| 0x26 | 7 | Water Speed with Average | Receive | Knauf/Roscoe |
| 0x27 | 4 | Precise Water Temperature | Receive | Knauf/Roscoe |
| 0x30 | 3 | Set Lamp Intensity | Transmit | Knauf/Roscoe |
| 0x36 | 3 | Cancel MOB | Transmit | Knauf/Roscoe |
| 0x38 | 4 | Codelock Data | Receive | Knauf |
| 0x50 | 5 | Latitude Position | Receive | Knauf/Roscoe |
| 0x51 | 5 | Longitude Position | Receive | Knauf/Roscoe |
| 0x52 | 4 | Speed Over Ground | Both | Knauf/Roscoe |
| 0x53 | 3 | Course Over Ground | Both | Knauf/Roscoe |
| 0x54 | 4 | GMT Time | Receive | Knauf/Roscoe |
| 0x56 | 4 | Date | Receive | Knauf/Roscoe |
| 0x57 | 3 | Satellite Info | Receive | Knauf/Roscoe |
| 0x58 | 8 | Lat/Lon Combined Position | Receive | Knauf/Roscoe |
| 0x59 | 5 | Set Countdown Timer | Transmit | Knauf/Roscoe |
| 0x66 | 3 | Wind Alarm | Both | Knauf/Roscoe |
| 0x68 | 4 | Alarm Acknowledgement | Both | Knauf/Roscoe |
| 0x6E | 10 | MOB Keystroke | Receive | Knauf |
| 0x70 | 3 | ST60 Remote Keystroke | Receive | Knauf |
| 0x80 | 3 | Set Lamp Intensity (alt) | Transmit | Knauf |
| 0x81 | 3/4 | Course Computer Setup | Receive | Knauf |
| 0x82 | 8 | Target Waypoint Name | Both | Knauf/Roscoe |
| 0x83 | 10 | Course Computer Failure | Receive | Knauf/Roscoe |
| **0x84** | **9** | **Autopilot Status** | **Receive** | **All** |
| 0x85 | 9 | Navigation to Waypoint | Receive | Knauf/Roscoe |
| **0x86** | **4** | **Autopilot Command** | **Transmit** | **All** |
| 0x87 | 3 | Set Autopilot Response Level | Both | Knauf/Roscoe |
| 0x88 | 6 | Autopilot Parameter | Receive | Knauf/Roscoe |
| 0x89 | 5 | Steering Compass Course | Both | Knauf/Roscoe |
| 0x90 | 5 | Device Identification | Both | Knauf |
| 0x91 | 3 | Set Rudder Gain / Response | Both | Knauf/Roscoe |
| 0x92 | 5 | Set Autopilot Parameter | Transmit | Knauf/Roscoe |
| 0x93 | 3 | Enter Autopilot Setup | Transmit | Knauf |
| 0x95 | 9 | Autopilot Status (setting mode) | Receive | Knauf |
| 0x99 | 3 | Compass Variation | Receive | Knauf/Roscoe |
| 0x9A | 12 | Version String | Receive | Knauf |
| **0x9C** | **4** | **Compass Heading & Rudder Position** | **Both** | **Knauf/Roscoe** |
| 0x9E | 15 | Waypoint Definition | Both | Knauf |
| 0xA1 | 16 | Destination Waypoint Info | Receive | Knauf |
| 0xA2 | 7 | Waypoint Arrival Notice | Receive | Knauf |
| 0xA4 | 5/9 | Broadcast Query / Identity | Both | Knauf |
| 0xA5 | 4/7/10/16 | GPS/DGPS Fix Info | Receive | Knauf/Roscoe |
| 0xA8 | 6 | Alarm Notice | Receive | Knauf |
| 0xAB | 6 | Alarm Notice | Receive | Knauf |

## Critical Messages for Autopilot

### 0x84 — Autopilot Status (9 bytes) — Receive Only

Broadcast periodically by the autopilot course computer (SmartPilot S3). Contains heading, target course, mode, rudder position, and alarms.

**Format**: `84 u6 vw xy 0z 0m rr ss tt`

| Byte | Variable | Description |
|------|----------|-------------|
| 0 | 0x84 | Command byte |
| 1 | `u` | High nibble: heading encoding + turn direction |
| 2 | `vw` | Low 6 bits: compass heading continuation. High 2 bits: target heading high bits |
| 3 | `xy` | Target heading low byte: `(target % 90) * 2` |
| 4 | `z` | Mode (low nibble only): 0=Standby, 2=Auto, 4=Wind/Vane, 8=Track |
| 5 | `m` | Alarms: bit 2=Off Course, bit 3=Wind Shift |
| 6 | `rr` | Rudder position in degrees (signed, positive = right) |
| 7 | `ss` | Display flags |
| 8 | `tt` | Computer type: 0x08=400G, 0x05=150(G) |

**Heading decoding** (`parse_autopilot_status` in `seatalk_datagram.c:1070`):
```c
compass_heading = ((u & 0x03) * 90) + ((vw & 0x3f) * 2) +
                  ((u & 0x0c) ? (((u & 0x0c) == 0x0c) ? 2 : 1) : 0);
turning_direction = (u & 0x08) ? RIGHT : LEFT;
target_heading = (((vw & 0xc0) >> 6) * 90) + (xy / 2);
rudder_position = fix_twos_complement(rr);
```

**Mode decoding**:
| z value | Mode |
|---------|------|
| 0x0 | Standby |
| 0x2 | Auto |
| 0x4 | Wind/Vane |
| 0x8 | Track |

**Alarm bits** (byte 5, low nibble `m`):
| Bit | Alarm |
|-----|-------|
| 0x04 | Off Course |
| 0x08 | Wind Shift |

**Example hex packet**: `84 26 A2 88 40 00 FE 02 06`
- compass heading: `u=2, vw=0xA2(162)` → `(2&3)*90 + (162&0x3f)*2 + ...` → heading ~180°
- target heading: `((162&0xc0)>>6)*90 + (0x88/2)` → heading ~180°
- mode = 0 (Standby)
- rudder = -2° (0xFE is -2 in two's complement)

### 0x86 — Autopilot Command (4 bytes) — Transmit

**Format**: `86 x1 yy !yy`

| Byte | Variable | Description |
|------|----------|-------------|
| 0 | 0x86 | Command byte |
| 1 | `x` | High nibble: remote type (1=Z101, 2=ST600R). Low nibble: always 1 (4-byte message) |
| 2 | `yy` | Sub-command (see table below) |
| 3 | `!yy` | Checksum = `yy ^ 0xFF` (bitwise complement) |

**Validation** (`parse_autopilot_command`):
```c
if ((yy | yy_comp) != 0xff) → checksum error
```

### 0x86 Sub-Command Table

| Hex | Constant | Function |
|-----|----------|----------|
| 0x01 | `ST_AUTOPILOT_COMMAND_AUTO` | Engage Auto mode |
| 0x02 | `ST_AUTOPILOT_COMMAND_STANDBY` | Disengage to Standby |
| 0x03 | `ST_AUTOPILOT_COMMAND_TRACK` | Engage Track mode |
| 0x04 | `ST_AUTOPILOT_COMMAND_DISP` | Cycle display |
| 0x05 | `ST_AUTOPILOT_COMMAND_TURN_LEFT_1` | -1 degree course change |
| 0x06 | `ST_AUTOPILOT_COMMAND_TURN_LEFT_10` | -10 degrees course change |
| 0x07 | `ST_AUTOPILOT_COMMAND_TURN_RIGHT_1` | +1 degree course change |
| 0x08 | `ST_AUTOPILOT_COMMAND_TURN_RIGHT_10` | +10 degrees course change |
| 0x09 | `ST_AUTOPILOT_COMMAND_DECREASE_GAIN` | Decrease rudder gain |
| 0x0A | `ST_AUTOPILOT_COMMAND_INCREASE_GAIN` | Increase rudder gain |
| 0x21 | `ST_AUTOPILOT_COMMAND_TACK_LEFT` | Tack port |
| 0x22 | `ST_AUTOPILOT_COMMAND_TACK_RIGHT` | Tack starboard |
| 0x23 | `ST_AUTOPILOT_COMMAND_WIND_MODE` | Engage Wind/Vane mode |
| 0x28 | `ST_AUTOPILOT_COMMAND_TRACK_MODE` | Engage Track mode (alt) |
| 0x2E | `ST_AUTOPILOT_COMMAND_TOGGLE_RESPONSE_LEVEL` | Toggle response display |
| 0x41 | `ST_AUTOPILOT_COMMAND_RETURN_TO_COURSE` | Resume previous course |
| 0x63 | `ST_AUTOPILOT_COMMAND_RETURN_TO_WIND_ANGLE` | Resume previous wind angle |
| 0x6E | `ST_AUTOPILOT_COMMAND_ENTER_RUDDER_GAIN_MODE` | Enter rudder gain display |
| 0x80–0x83 | `HOLD_*` | Held key (repeated every second) |
| 0x84 | `ST_AUTOPILOT_COMMAND_RELEASE_HELD_KEY` | Release held key |

**Complete command packets** (from AK-Homberger):

| Command | Byte 0 | Byte 1 | Byte 2 | Byte 3 |
|---------|--------|--------|--------|--------|
| Auto | 0x86 | 0x21 | 0x01 | 0xFE |
| Standby | 0x86 | 0x21 | 0x02 | 0xFD |
| +1° | 0x86 | 0x21 | 0x07 | 0xF8 |
| -1° | 0x86 | 0x21 | 0x05 | 0xFA |
| +10° | 0x86 | 0x21 | 0x08 | 0xF7 |
| -10° | 0x86 | 0x21 | 0x06 | 0xF9 |
| Track | 0x86 | 0x21 | 0x03 | 0xFC |
| Wind | 0x86 | 0x21 | 0x23 | 0xDC |

### 0x9C — Compass Heading & Rudder Position (4 bytes) — Transmit

**Format**: `9C u1 vw rr`

Used to transmit heading and rudder data to the bus (e.g., from a fluxgate compass or rudder sensor).

| Byte | Content |
|------|---------|
| 0 | 0x9C |
| 1 | High nibble: heading encoding (`u`). Low nibble: always 1 (4-byte msg) |
| 2 | `vw`: Low 6 bits = heading continuation. High 2 bits reserved |
| 3 | `rr`: Rudder position (signed), degrees right positive |

Heading decoding (same algorithm as 0x84):
```c
heading = (((u & 0x3) * 90) + ((vw & 0x3f) * 2) +
           (u & 0xc ? ((u & 0xc) == 0xc ? 2 : 1) : 0)) % 360;
```

### 0x89 — Steering Compass Course (5 bytes) — Transmit

**Format**: `89 u2 VW XY 2Z`

| Byte | Content |
|------|---------|
| 0 | 0x89 |
| 1 | High nibble: heading bits |
| 2 | `VW`: Low 6 bits = heading. High 2 bits = locked heading high bits |
| 3 | `XY`: Locked heading × 2 |
| 4 | Low nibble: bit 1 = locked steer active, bit 5 always set |

## Sensor Messages

### 0x10 — Apparent Wind Angle (4 bytes)

**Format**: `10 01 xx yy`

Scaling: angle = `(xxyy / 2)` degrees right of bow (signed, 0 = head-to-wind)

### 0x11 — Apparent Wind Speed (4 bytes)

**Format**: `11 01 xx 0y`

Speed: `(xx & 0x7f) * 10 + y` knots (or m/s if `xx & 0x80`)

### 0x20 — Water Speed (4 bytes)

**Format**: `20 01 xx xx`

Scaling: `xxxx / 10` knots

### 0x00 — Depth Below Transducer (5 bytes)

**Format**: `00 02 yz xx xx`

Scaling: `xxxx / 10` feet. `y` bits: anchor alarm, metric. `z` bits: deep/shallow alarm, transducer fault.

### 0x52 — Speed Over Ground (4 bytes)

**Format**: `52 01 xx xx`

Scaling: `xxxx / 10` knots

### 0x53 — Course Over Ground (3 bytes)

**Format**: `53 u0 vw`

Degrees: `(u & 0x3) * 90 + (vw & 0x3f) * 2 + (u >> 3)`

## Parameter Commands

### 0x92 — Set Autopilot Parameter (5 bytes) — Transmit

**Format**: `92 02 <param> <value> 00`

| Parameter | Code | Description |
|-----------|------|-------------|
| Response level | 0x12 | Sets autopilot response (1-9) |
| Wind trim | 0x11 | Sets wind trim (1-9) |
| Rudder gain | 0x01 | Sets rudder gain (1-9) |

### 0x87 — Set Autopilot Response Level (3 bytes) — Both

**Format**: `87 00 0x`

`x`: 0 = automatic deadband, 1 = minimum deadband (or 1-9 response level from other sources)

### 0x91 — Set Rudder Gain (3 bytes) — Both

**Format**: `91 00 0x`

`x`: rudder gain value (1-9)

## Alarm Commands

### 0xA8 / 0xAB — Alarm Notice (6 bytes)

**Format**: `A8 03 xx yy zz ww`

Used by Guard #1 / Guard #2 alarm systems. Byte 2 identifies alarm type.

### 0x68 — Alarm Acknowledgement (4 bytes) — Transmit

**Format**: `68 x1 yy 00`

`x` identifies alarm type (1=shallow, 2=deep, 3=anchor, 4-11=wind alarms). `yy` = acknowledging device ID (hardcoded to 0x01).

### 0x66 — Wind Alarm (3 bytes)

**Format**: `66 00 xy`

`x` high nibble: apparent wind alarms. `y` low nibble: true wind alarms. Bits: 8=angle low, 4=angle high, 2=speed low, 1=speed high.

## Navigation Messages

### 0x82 — Target Waypoint Name (8 bytes)

**Format**: `82 05 xx !xx yy !yy zz !zz`

Four 6-bit ASCII characters encoded across 6 bytes with complement checksums. Characters must be uppercase, subtract 0x30 before encoding.

### 0x85 — Waypoint Navigation Status (9 bytes)

**Format**: `85 x6 xx vu zw zz yf 00 !yf`

Contains cross-track error, bearing to waypoint, distance, and direction to steer. Byte 8 is complement checksum of byte 6.

### 0x58 — Lat/Lon Position (8 bytes)

**Format**: `58 z5 la xx yy lo qq rr`

Latitude: `la` degrees, `xxyy / 1000` minutes. Longitude: `lo` degrees, `qqrr / 1000` minutes. `z` bit 0 = south, bit 1 = east.

### 0x9E — Waypoint Definition (15 bytes) — Transmit

Used to define a waypoint on the bus. Complex multi-field format.

## Utility Messages

### 0x30 / 0x80 — Set Lamp Intensity (3 bytes)

**Format**: `30 00 0x` or `80 00 0x`

`x` = intensity: 0=off, 4=low, 8=medium, 0xC=high

### 0x99 — Compass Variation (3 bytes)

**Format**: `99 00 xx`

`xx` = variation in degrees west (signed; negative = east)

### 0x9A — Version String (12 bytes)

Contains product version information broadcast by devices on power-up.

### 0x90 — Device Identification (5 bytes — variable length)

**Format**: `90 z2 xx yy ...`

`z` identifies device type. Used on power-up or in response to 0xA4 query.

### 0xA4 — Broadcast Query (5 or 9 bytes)

Sent by a device to request all other devices on the bus to identify themselves. Devices respond with 0x90.

### 0x38 — Codelock Data (4 bytes)

Used by Raymarine ST60+ instruments with code lock feature.

## NMEA Bridge ID (AK-Homberger specific)

**Format**: `90 00 A3` (3 bytes)

Sent every 10 seconds by the AK-Homberger remote to maintain compatibility with SeaTalk-NG converters. From the code:
```cpp
const uint16_t ST_NMEA_BridgeID[] = { 0x190, 0x00, 0xA3 };
```
The 9th bit is set on byte 0 (0x190 = 0x90 with bit 8 set).

## Implemented in AK-Homberger Project Only

| Command | Length | Description |
|---------|--------|-------------|
| 0x90 | 3 | NMEA Bridge ID (0x190 with 9-bit flag) |
| 0xA8 | 6 | Alarm control (beep on/off via 0x1A8 with 9-bit) |

The beep commands use a non-standard format:
```
Beep On:  0xA8 0x53 0x80 0x00 0x00 0xD3
Beep Off: 0xA8 0x43 0x80 0x00 0x00 0xC3
```

## Units and Scaling Reference

| Measurement | Raw Encoding | Real-World Value | Notes |
|-------------|-------------|-----------------|-------|
| Heading | integer | degrees (0-359) | 3-byte compass encoding |
| Depth | `xxxx` | `xxxx / 10` feet | 0.1 ft resolution |
| Wind angle | `xxyy` | `xxyy / 2` degrees | signed |
| Wind speed | `(xx&0x7f)*10 + y` | knots (or m/s) | 0.1 kt resolution |
| Water speed | `xxxx` | `xxxx / 10` kt | 0.1 kt resolution |
| SOG | `xxxx` | `xxxx / 10` kt | 0.1 kt resolution |
| Trip log | `xxxxx` | `xxxxx / 100` nm | 0.01 nm resolution |
| Total log | `xxxx` | `xxxx / 10` nm | 0.1 nm resolution |
| Temperature | `xxxx` | `(xxxx-100)/10` °C | 0.1 °C resolution |
| Rudder | byte | degrees (signed) | 1° resolution |
| RPM | `yyzz` | RPM | 16-bit signed |
| Latitude | deg + min | `dd mm.mmm` | minutes × 1000 |
| Longitude | deg + min | `ddd mm.mmm` | minutes × 1000 |
