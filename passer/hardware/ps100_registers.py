"""PS100 RS-485 / Modbus register map, from the manuals in docs/manuals/.

Addressing (group base + parameter number) and the REQUIRED_SETTINGS values
were checked against the real drive; see docs/ps100.md. Scaling and
gearbox maths live in kinematics.py / LauncherConfig, not here.
"""

# ----------------------------
# Communication defaults
# ----------------------------

DEFAULT_SLAVE_ID = 1
DEFAULT_BAUDRATE = 9600

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

# Command pulses per motor revolution the code assumes (PA11, checked by `check`).
PULSES_PER_REV = 10_000

# ----------------------------
# Status / monitoring registers (read-only; RS485 manual section 1.5)
# ----------------------------

REG_MOTOR_SPEED = 0x1000
# Positions below are in ENCODER counts (17-bit: 131072 per motor rev),
# not the 10000 command pulses per rev used for moves.
REG_POSITION = 0x1001             # 2 words, low first: current position
REG_POSITION_COMMAND = 0x1003     # 2 words: position command
REG_POSITION_ERROR = 0x1005       # 2 words: following error
REG_CONTROL_MODE = 0x1009         # same codes as PA4
REG_INPUT_STATUS = 0x100F         # DI terminals, bit 0 = DI1 (virtual inputs show here too)
REG_OUTPUT_STATUS = 0x1010        # DO terminals, bit 0 = DO1
POSITION_COMPLETE_BIT = 0         # DO1 when P3-20 = 16; the manual says watch it *change*
REG_BUS_VOLTAGE = 0x1012
REG_ALARM = 0x1013                # current alarm number, 0 = none (codes in ALARM_CODES)
REG_RUN_STATE = 0x1016
REG_ABS_POSITION = 0x1018         # 4 words, low first: encoder absolute position (P3-37 = 0)

# (first register, words, label) for `ps100_cli monitor`, read in one request.
MONITOR = [
    (0x1000, 1, "motor speed, rpm"),
    (0x1001, 2, "current position, encoder counts"),
    (0x1003, 2, "position command, encoder counts"),
    (0x1005, 2, "position error, encoder counts"),
    (0x1007, 1, "motor torque"),
    (0x1008, 1, "motor current"),
    (0x1009, 1, "control mode (PA4 codes: 0 = position)"),
    (0x100A, 1, "temperature"),
    (0x100B, 1, "speed command"),
    (0x100C, 1, "torque command"),
    (0x100D, 2, "rotor position within one turn"),
    (0x100F, 1, "input terminals DI4..DI1"),
    (0x1010, 1, "output terminals DO4..DO1"),
    (0x1011, 1, "encoder input signal"),
    (0x1012, 1, "bus voltage"),
    (0x1013, 1, "alarm (0 = none)"),
    (0x1014, 1, "logic chip version"),
    (0x1015, 1, "relay state"),
    (0x1016, 1, "running state (codes not documented)"),
    (0x1017, 1, "external voltage state"),
    (0x1018, 4, "encoder absolute position"),
]
MONITOR_SPAN = (0x1000, 0x101C)   # [start, end): one read covers MONITOR

# ----------------------------
# Parameter addressing
# ----------------------------

# address = group base + parameter number (decimal), RS485 manual 1.4.
# Confirmed on the drive. The PA group shows as "FA" on the panel, and the
# quick manual's "FC-n" input codes are P3-n. Writing an address saves to
# the drive's EEPROM; PA parameters also have a RAM-only copy at +0x0080
# (manual 1.6). No RAM-only address is documented for P3/P4.
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
    # 28, as in the manual's push-button version of this move. Its RS-485
    # sections say 38, which the drive accepts but never starts a move with.
    ("P3-38", 28, "virtual DI1 function: 28 = internal-position start"),
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
    ("PA22", "speed command source: 3 = panel jog setup; factory default 0"),
    ("PA23", "maximum motor speed, rpm"),
    ("PA40", "acceleration time, ms for 0 -> 1000 rpm"),
    ("PA41", "deceleration time, ms for 1000 -> 0 rpm"),
    ("PA74", "on comms error: 0 = keep running, 1 = alarm and stop"),
    ("P4-2", "loaded move: whole turns"),
    ("P4-3", "loaded move: pulses"),
    ("P4-4", "loaded move: speed, rpm"),
]

# Alarm numbers (quick manual, 报警代码 table): shown on the display and
# readable from REG_ALARM.
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

