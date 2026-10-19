# PS100 RS-485 Configuration for the Basketball Passing Machine

## Purpose

This document records the proposed PS100 servo-drive configuration for the project.

The intended control architecture is:

```text
Player tracking / throw planner
            |
            v
      Raspberry Pi
            |
       USB to RS-485
            |
            v
        PS100 drive
            |
            v
       Servo motor
            |
            v
 Carbon-fibre throwing arm
```

For the initial implementation, the Raspberry Pi calculates a desired arm displacement and speed, sends those values to the PS100 over Modbus RTU, and then triggers one internal-position movement.

The PS100 performs the actual servo-control loop internally.

---

## 1. Recommended Control Mode

Use **RS-485 internal-position control, relative-position single-segment mode**.

This mode is documented in the supplied advanced-use manual under:

> 485 通讯方式控制（相对位置单段模式）

The important runtime registers are:

| Register | Drive parameter | Purpose |
|---|---|---|
| `0x0202` | P4-2 | Number of revolutions for first internal-position segment |
| `0x0203` | P4-3 | Pulses within the revolution |
| `0x0204` | P4-4 | Movement speed |
| `0x011F` | P3-31 | Virtual-input control / motion trigger |
| `0x1010` | status | Can be used to observe positioning-complete status after DO1 is configured |

The manual states that P4-2 and P4-3 may contain positive or negative values, while P4-4 must be positive.

---

## 2. PS100 Parameters

Configure the drive approximately as follows.

| Parameter | Value | Purpose |
|---|---:|---|
| PA/FA4 | `0` | Position-control mode |
| PA/FA11 | `10000` | 10,000 command pulses per motor revolution |
| PA/FA14 | `3` | Internal-position input |
| P3-30 | `1` | Enable virtual-input-terminal control |
| P3-38 | `38` | Virtual DI1 = internal-position start |
| PA/FA53 | `1` | Forced/internal servo enable |
| P4-0 | `1` | Relative internal-position mode |
| PA/FA71 | `1` | Example Modbus slave address |
| PA/FA72 | `96` | 9600 baud |
| PA/FA73 | `3` | 8 data bits, no parity, 1 stop bit |

### Documentation naming note

The supplied documents use different prefixes such as `PA`, `FA`, `P3`, and `FC` in different places/drive variants. Use the parameter naming actually displayed by the specific PS100 drive/firmware.

---

## 3. RS-485 Communication Settings

For the proposed configuration:

```text
Protocol:       Modbus RTU
Slave address:  1
Baud rate:      9600
Data bits:      8
Parity:         None
Stop bits:      1
```

The manual documents the communication parameters as:

- PA/FA71: Modbus slave address, range 1-254.
- PA/FA72: baud-rate value in units of 100; `96` therefore represents 9600 baud.
- PA/FA73: Modbus RTU format.

For PA/FA73:

```text
0 = 8N2
1 = 8E1
2 = 8O1
3 = 8N1
```

This project proposes `3` (8N1).

---

## 4. Physical RS-485 Connection

The supplied PS100 quick manual shows the communication connector with:

```text
Pin 4 = RS485-
Pin 5 = RS485+
Pin 7 = GND
```

A typical Raspberry Pi connection is:

```text
Raspberry Pi
    |
    USB
    |
USB-RS485 adapter
    |
    +---------------- PS100 RS485+
    +---------------- PS100 RS485-
    +---------------- PS100 GND
```

Check the USB-RS485 adapter's A/B or +/- convention carefully because RS-485 naming conventions are not always consistent between manufacturers.

Do not confuse the PS100 RS-485 pins with the CAN H/CAN L pins located on the same communication connector.

---

## 5. Position Scaling

With:

```text
PA/FA11 = 10000
```

one motor revolution corresponds to 10,000 command pulses:

```text
360 deg = 10,000 pulses
```

Therefore:

```text
pulses = degrees / 360 * 10000
```

Examples:

| Angle | Approx. pulses |
|---:|---:|
| 30 deg | 833 |
| 90 deg | 2500 |
| 150 deg | 4167 |
| 180 deg | 5000 |
| 360 deg | 10000 |

These values refer to **motor-shaft command position**. If the throwing arm uses gearing, belts, or another transmission ratio, the software must account for that ratio.

---

## 6. Issuing a Relative Position Move

Example requirement:

```text
Move:  +150 degrees
Speed: 30 rpm
```

With 10,000 pulses/revolution:

```text
150 / 360 * 10000 ~= 4167 pulses
```

The Raspberry Pi would conceptually perform:

```text
0x0202 <- 0       # complete turns
0x0203 <- 4167    # remaining pulses
0x0204 <- 30      # position movement speed

0x011F <- 0
0x011F <- 1       # rising edge triggers movement
```

The advanced-use manual explicitly describes the internal-position command as edge-triggered:

```text
0x011F: 0000 -> 0001
```

After the trigger, the servo drive executes the configured movement.

---

## 7. Detecting Motion Completion

The manual provides an optional way to detect completion of an internal-position movement.

Configure:

```text
P3-20 = 16
```

This defines DO1 as:

```text
Internal position positioning complete
```

The manual then says that the controller can read:

```text
0x1010
```

and observe a change in **bit 0**.

This can be used by the Raspberry Pi to implement:

```text
SEND MOVE
    |
    v
SERVO MOVING
    |
    | poll 0x1010
    v
POSITION COMPLETE
    |
    v
NEXT STATE
```

### Important ambiguity

The supplied manual says to observe a **change** in bit 0, but does not clearly state in the supplied text whether:

```text
bit 0 = 1
```

always means "motion complete", or whether software should specifically detect a transition.

Verify this experimentally before relying on it.

---

## 8. Optional Immediate Stop

The advanced-use manual states that an additional virtual input can be configured:

```text
P3-39 = 27
```

to define DI2 as an internal-position stop input.

With that configuration:

```text
0x011F = 0x0002
```

causes the motor to stop the internal-position movement immediately.

This may be useful as a software stop command.

It should **not** be treated as a replacement for a properly designed hardware emergency-stop/safety system.

---

## 9. Acceleration and Deceleration

The PS100 manual documents:

```text
PA/FA40 = acceleration time
PA/FA41 = deceleration time
PA/FA42 = S-curve acceleration/deceleration time
```

FA40 is described as the time for the motor to accelerate from 0 to 1000 r/min. FA41 applies to deceleration from 1000 r/min to zero.

The manual states that these parameters apply to internal-position control.

For a throwing mechanism, these parameters are important because the ball's release velocity depends on the arm's actual angular velocity at the release point, not merely the final commanded position.

These values should therefore be tuned experimentally and conservatively.

---

## 10. Proposed Software Architecture

Do not initially use Modbus as a high-frequency servo control loop.

Instead:

```text
CAMERA / PLAYER TRACKING
          |
          v
Estimate player position + velocity
          |
          v
Predict catch point
          |
          v
Calculate desired throw
          |
          v
angle + servo speed
          |
          v
       FIRE?
          |
          v
Freeze selected throw parameters
          |
          v
Write 0x0202 / 0x0203 / 0x0204
          |
          v
Trigger 0x011F: 0 -> 1
          |
          v
PS100 executes motion internally
          |
          v
Poll 0x1010 for completion
```

The Raspberry Pi can continuously update the *planned* throw while tracking the player. Once a throw is committed, the initial design should send one coherent position/speed command and allow the PS100 to execute it.

The supplied documentation does not specify how rewriting `0x0202`, `0x0203`, or `0x0204` during an already-triggered move affects the active trajectory. Do not depend on that behaviour without testing or manufacturer confirmation.

---

## 11. Python Constants

The accompanying file:

```text
passer/hardware/ps100_registers.py
```

contains the register addresses and scaling constants so other programs can import them:

```python
from passer.hardware.ps100_registers import (
    REG_POSITION_TURNS,
    REG_POSITION_PULSES,
    REG_POSITION_SPEED,
    REG_VIRTUAL_INPUT_CONTROL,
    REG_OUTPUT_STATUS,
    PULSES_PER_REV,
)
```

Example:

```python
from passer.hardware.ps100_registers import degrees_to_pulses

target = degrees_to_pulses(150)
print(target)  # approximately 4167
```

Keeping register addresses in one module avoids scattering unexplained hexadecimal values throughout the project.

The driver that actually performs the write sequence is `passer/hardware/ps100.py`
(`PS100.move(turns, pulses, rpm)`), and `passer/kinematics.py` converts arm
degrees / arm rpm into these register values through the 10:1 gearbox. For bench
testing use:

```bash
python -m passer.tools.ps100_cli --dry-run arm 90 --rpm 60   # print writes only
python -m passer.tools.ps100_cli status                      # read 0x1010
python -m passer.tools.ps100_cli arm 10 --rpm 30             # small real move
```

---

## 12. Items Still Requiring Verification

Before commanding significant motion, verify these points on the actual hardware:

1. The exact parameter names shown by the specific PS100 firmware.
2. The USB-RS485 adapter's +/- or A/B polarity.
3. The exact Modbus register-address convention used by the drive and Python library.
4. Which Modbus read/write function codes the drive expects.
5. The behaviour of `0x1010` bit 0 before, during, and after positioning.
6. Whether runtime Modbus writes to the position registers are RAM-only or otherwise affect nonvolatile storage.
7. Safe acceleration, speed, torque, and mechanical travel limits for the throwing mechanism.

Start testing with the throwing arm mechanically unloaded or otherwise made safe, at low speed and small displacement.

---

## 13. Parameter Addressing

Every parameter can be read (and written) over Modbus. The manuals give
addresses for a handful of parameters, and all of them follow one rule:

```text
address = group base + parameter number (decimal)

PA (FA) group  base 0x0000    PA14  -> 0x000E
P3 group       base 0x0100    P3-38 -> 0x0126
P4 group       base 0x0200    P4-0  -> 0x0200
```

Documented pairs that confirm it: PA21 → `0x0015`, PA24 → `0x0018`,
PA25 → `0x0019`, PA50 → `0x0032`, PA64 → `0x0040`, PA65 → `0x0041`,
P3-31 → `0x011F`, P3-34 → `0x0122`, P4-2..P4-4 → `0x0202..0x0204`.
Addresses for other parameters are inferred from the rule.

`passer.hardware.ps100_registers.param_address("P3-38")` applies it, and
`ps100_cli read P3-38` reads a parameter by name.

Settings the RS-485 move depends on (`ps100_cli check` reads all of them):

| Parameter | Address | Needs | Meaning |
|---|---|---:|---|
| PA4 | `0x0004` | 0 | Control mode: position |
| PA11 | `0x000B` | 10000 | Command pulses per motor revolution |
| PA14 | `0x000E` | 3 | Position command source: internal position |
| PA53 | `0x0035` | 1 | Servo forced on (no external enable wire) |
| P3-30 | `0x011E` | 1 | Virtual input terminals controlled by `0x011F` |
| P3-38 | `0x0126` | 38 | Virtual DI1 = internal-position start |
| P3-39 | `0x0127` | 27 | Virtual DI2 = internal-position stop |
| P3-20 | `0x0114` | 16 | DO1 = positioning complete (bit 0 of `0x1010`) |
| P4-0 | `0x0200` | 1 | Internal-position mode: relative (0 = absolute) |
| PA71 | `0x0047` | 1 | Modbus slave address |
| PA72 | `0x0048` | 96 | Baud rate / 100 |
| PA73 | `0x0049` | 3 | Frame format 8N1 |

Also worth reading when a move misbehaves: PA20 (`0x0014`, drive-prohibit
inputs; 1 = ignored), PA23 (`0x0017`, max speed), PA40/PA41 (`0x0028`/`0x0029`,
ramps), PA74 (`0x004A`, on comms error: 0 = keep running, 1 = alarm and stop),
and P4-2..P4-4 (the move currently loaded).

`0x011F` is a bit field over the virtual inputs, `DI4 DI3 DI2 DI1`. The
manual writes the values as bit strings: `0001` = DI1 (start), `0010` = DI2
(stop). The code writes `0x0001` and `0x0002`.

## 14. Status and Monitoring

| What | Where | Notes |
|---|---|---|
| Positioning complete | `0x1010` bit 0 | Only with P3-20 = 16. The manual says to watch for a *change*; see §7 |
| Encoder absolute position | `0x1018..0x101B` | 64-bit, low word first (with P3-37 = 0). `ps100_cli pos` reads it. Raw encoder units: compare readings to see whether the motor moved |
| Clear encoder position | write 0 to `0x0122` (P3-34) | Needs P3-36 = 1 to clear single- and multi-turn. Not used by the code |
| Motor speed, current, pulses, position | Panel `dP-` menu: `F-spd`, `F-I`, `F-cpo`, `F-apo` | Front panel only |
| Current alarm | Panel display | **No Modbus register is documented** for the alarm number or servo-on state |

`ps100_cli arm` prints the encoder position before and after the move, so a
missing completion signal can be told apart from a motor that never moved.

## 15. Alarm Codes

From the quick manual's alarm table (报警代码). `ps100_cli alarm <n>` prints
the same text. Clear an alarm by fixing its cause, then power-cycle (or set
PA61 = 1).

| No. | Alarm | Likely cause here |
|---:|---|---|
| 1 | Overspeed | Speed above limit |
| 2 / 3 | Main circuit over / undervoltage | Supply problem |
| 4 | Position following error > PA17 | Arm too heavy for the ramp or speed, or gains too low |
| 5 | Drive overheated | |
| 6 | Speed amplifier saturated | Load too heavy for the commanded speed |
| 7 | Drive-prohibit inputs both off | Set PA20 = 1 to ignore the limit inputs |
| 8 | Position counter overflow | |
| 11 | IPM module fault | Power stage |
| 13 | Drive/motor overload | |
| 14 | Braking circuit fault | Check the P–D jumper / brake resistor |
| 18 | Relay state mismatch | |
| 19 | Brake delay error | Pulses while the brake was closed |
| 20 | EEPROM error | |
| 21 | FPGA fault | |
| 23 | Current sensing fault | |
| 29 | User torque overload | PA30 / PA31 |
| 38 | Encoder read/write failed | Encoder cable |
| 39 | Encoder data CRC error | |
| 40 / 41 | Motor model unsupported / differs | |
| 42 | AC input too low | |
| 47 | Main voltage too high at power-up | |
| 50 / 51 | No / lost encoder communication | Encoder cable |
| 52 | Encoder battery low | |
| 53 | Encoder battery error, data lost | Normal on first connection of a multi-turn motor: clear with `F-c r` |
| 54 / 59 | Encoder error | Reset the encoder |
| 55 | Encoder CRC failed 3 times | |
| 56 | Modbus frame too long | |
| 57 | Modbus format error | Comms settings, register address or value not accepted |
| 58 | Single-turn offset out of range | |

## 16. When a Move Fails

1. **Look at the drive display.** If it shows an alarm number, look it up in §15.
2. **`ps100_cli status`.** If this times out, it's the link: wiring (A/B swap), PA71–PA73, `ps100.port`.
3. **`ps100_cli check`.** Fix every `FIX` line on the panel, save (long-press SET), and power-cycle. P3-30 = 1 and P3-38 = 38 are the usual missing pair: without them the trigger write is accepted but nothing moves.
4. **`ps100_cli read P4-4`** after a move. If it doesn't show the speed you sent, writes are landing on the wrong address: try `ps100.address_offset: -1`.
5. **`ps100_cli arm 10 --rpm 30`.** Read the result line:
   - `done`: working.
   - `motor moved but no completion signal`: set P3-20 = 16, or `ps100.completion_mode: time`.
   - `motor did not move`: settings or alarm (steps 1 and 3), or the servo isn't enabled (PA53).
