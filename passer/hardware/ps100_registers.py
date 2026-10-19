"""PS100 RS-485 / Modbus register constants.

Based on the supplied PS100/KS100 manuals for the intended use case:
relative internal-position control over RS-485.

Important:
- The manuals document these hexadecimal register addresses, but do not
  clearly specify whether a particular Modbus library requires an address
  offset. Verify addressing on the actual drive before motion testing.
- Register names here follow the functions described in the manuals.
"""

# ----------------------------
# Communication defaults
# ----------------------------

DEFAULT_SLAVE_ID = 1
DEFAULT_BAUDRATE = 9600
DEFAULT_BYTESIZE = 8
DEFAULT_PARITY = "N"
DEFAULT_STOPBITS = 1

# ----------------------------
# Position scaling
# ----------------------------

# Corresponds to PA/FA11 = 10000 in the proposed configuration.
PULSES_PER_REV = 10_000
DEGREES_PER_REV = 360.0

# Launch arm gearbox: the motor turns this many times per arm revolution.
GEAR_RATIO = 10

# ----------------------------
# Runtime Modbus registers
# ----------------------------

# P3-31: virtual input terminal control word.
# In relative internal-position mode, a 0 -> 1 transition triggers motion.
REG_VIRTUAL_INPUT_CONTROL = 0x011F

# P4-2: first-segment internal-position revolution count.
# May be positive or negative.
REG_POSITION_TURNS = 0x0202

# P4-3: first-segment position within a revolution, in command pulses.
# May be positive or negative.
REG_POSITION_PULSES = 0x0203

# P4-4: first-segment internal-position speed.
# Manual states this value must be positive.
REG_POSITION_SPEED = 0x0204

# Status register used for virtual/output status.
# With P3-20 configured as 16 (DO1 = internal positioning complete),
# the manual says to observe bit 0 of this register for a state change.
REG_OUTPUT_STATUS = 0x1010
POSITION_COMPLETE_BIT = 0

# ----------------------------
# Optional absolute encoder registers
# ----------------------------

# Absolute-position words, low word first.
REG_ABS_POSITION_WORD_0 = 0x1018  # bits 15..0
REG_ABS_POSITION_WORD_1 = 0x1019  # bits 31..16
REG_ABS_POSITION_WORD_2 = 0x101A  # bits 47..32
REG_ABS_POSITION_WORD_3 = 0x101B  # bits 63..48

# P3-34 register documented for encoder-coordinate clearing.
REG_ENCODER_COORDINATE_CLEAR = 0x0122

# ----------------------------
# Parameter addressing
# ----------------------------

# Every address the manuals give for a parameter follows one rule:
# group base + parameter number (decimal). Checked against all documented
# pairs: PA21 -> 0x0015, PA24 -> 0x0018, PA25 -> 0x0019, PA50 -> 0x0032,
# PA64 -> 0x0040, PA65 -> 0x0041, P3-31 -> 0x011F, P3-34 -> 0x0122,
# P4-2..P4-4 -> 0x0202..0x0204. Other parameters are inferred from it.
# The PA group is shown as "FA" on some firmware; the quick manual's
# "FC-n" input codes correspond to P3-n.
PARAM_GROUP_BASE = {"PA": 0x0000, "FA": 0x0000, "P3": 0x0100, "FC": 0x0100,
                    "P4": 0x0200}


def param_address(name: str) -> int:
    """Modbus address of a drive parameter: 'PA14', 'FA14', 'P3-38', 'P4-0'."""
    key = name.strip().upper().replace(" ", "")
    for group in sorted(PARAM_GROUP_BASE, key=len, reverse=True):
        if key.startswith(group):
            number = key[len(group):].lstrip("-")
            if number.isdigit():
                return PARAM_GROUP_BASE[group] + int(number)
    raise ValueError(f"not a PS100 parameter name: {name!r} (try PA14, P3-38, P4-0)")


# Settings the relative internal-position RS-485 mode depends on
# (manual section 9, plus the optional stop and completion outputs).
# (parameter, expected value or None = just report it, what it does)
REQUIRED_SETTINGS = [
    ("PA4", 0, "control mode: 0 = position"),
    ("PA11", PULSES_PER_REV, "command pulses per motor revolution"),
    ("PA14", 3, "position command source: 3 = internal position"),
    ("PA53", 1, "servo enable: 1 = forced on (no external enable wire)"),
    ("P3-30", 1, "virtual input terminals: 1 = controlled by 0x011F"),
    ("P3-38", 38, "virtual DI1 function: 38 = internal-position start"),
    ("P3-39", 27, "virtual DI2 function: 27 = internal-position stop (software stop)"),
    ("P3-20", 16, "DO1 function: 16 = positioning complete (bit 0 of 0x1010)"),
    ("P4-0", 1, "internal-position mode: 1 = relative"),
    ("PA71", DEFAULT_SLAVE_ID, "Modbus slave address"),
    ("PA72", DEFAULT_BAUDRATE // 100, "baud rate / 100"),
    ("PA73", 3, "frame format: 3 = 8N1"),
]

# Worth seeing when a move misbehaves; no single right value.
INFO_SETTINGS = [
    ("PA20", "drive-prohibit inputs: 1 = ignored (0 can cause alarm 7)"),
    ("PA23", "maximum motor speed, rpm"),
    ("PA40", "acceleration time, ms for 0 -> 1000 rpm"),
    ("PA41", "deceleration time, ms for 1000 -> 0 rpm"),
    ("PA74", "on comms error: 0 = keep running, 1 = alarm and stop"),
    ("P4-2", "loaded move: whole turns"),
    ("P4-3", "loaded move: pulses"),
    ("P4-4", "loaded move: speed, rpm"),
]

# Alarm numbers shown on the drive display (quick manual, 报警代码 table).
# The manuals give no Modbus register for the current alarm: read it on
# the front panel.
ALARM_CODES = {
    1: "overspeed: motor exceeded the speed limit",
    2: "main circuit overvoltage",
    3: "main circuit undervoltage",
    4: "position following error exceeded PA17 (load too heavy, ramps too short, or gains too low)",
    5: "drive overheated",
    6: "speed amplifier saturated for too long",
    7: "drive-prohibit inputs CCW/CW both off (set PA20 = 1 to ignore them)",
    8: "position deviation counter overflow",
    11: "IPM power module fault",
    13: "drive/motor overload",
    14: "braking circuit fault",
    18: "relay state does not match its command",
    19: "brake delay error: pulses received while the brake was still closed",
    20: "EEPROM error",
    21: "FPGA fault",
    23: "current sensing circuit fault",
    29: "user torque overload (PA30/PA31)",
    38: "encoder read/write failed: encoder cable or interface",
    39: "encoder data CRC error (encoder not yet written)",
    40: "motor model not supported by this drive",
    41: "motor model differs from the one selected",
    42: "AC input voltage too low",
    47: "main circuit voltage too high at power-up",
    50: "no communication with the encoder",
    51: "encoder communication dropped after connecting",
    52: "encoder battery low (data kept; replace soon)",
    53: "encoder battery voltage error: data lost, reset the encoder (normal on first connection of a multi-turn motor)",
    54: "encoder error: reset the encoder",
    55: "encoder CRC failed 3 times in a row",
    56: "Modbus frame too long",
    57: "Modbus format error: comms settings, address or value not accepted",
    58: "stored single-turn offset exceeds encoder resolution",
    59: "encoder reported CF errors repeatedly: reset the encoder",
}

# ----------------------------
# Control-word values
# ----------------------------

CONTROL_IDLE = 0x0000
CONTROL_POSITION_TRIGGER = 0x0001

# Optional stop function if P3-39 is configured as function 27.
CONTROL_POSITION_STOP = 0x0002


def degrees_to_pulses(degrees: float) -> int:
    """Convert motor-shaft degrees to command pulses."""
    return round(degrees / DEGREES_PER_REV * PULSES_PER_REV)


def pulses_to_degrees(pulses: int) -> float:
    """Convert command pulses to motor-shaft degrees."""
    return pulses / PULSES_PER_REV * DEGREES_PER_REV


def arm_degrees_to_pulses(arm_degrees: float, gear_ratio: float = GEAR_RATIO) -> int:
    """Convert launch-arm degrees to motor command pulses through the gearbox."""
    return round(arm_degrees / DEGREES_PER_REV * gear_ratio * PULSES_PER_REV)
