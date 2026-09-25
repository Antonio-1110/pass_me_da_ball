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
