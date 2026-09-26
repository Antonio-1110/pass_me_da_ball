"""
ps100.py

PS100 servo drive control over Modbus RTU / RS-485 using relative-position
single-segment mode (see PS100_RS485_CONFIGURATION.md).

One throw is:

    write 0x0202 <- whole motor turns      (signed)
    write 0x0203 <- remaining pulses       (signed)
    write 0x0204 <- motor speed in rpm     (> 0)
    write 0x011F <- 0x0000                 (make sure the next write is an edge)
    write 0x011F <- 0x0001                 (rising edge = go)
    poll  0x1010 bit 0 until positioning-complete (optional)

Two transports are provided:
  * ModbusTransport  - real hardware via minimalmodbus + USB-RS485 adapter.
  * SimTransport     - in-memory fake for dry runs and tests; it logs every
                       write and flips the status bit like the drive would.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, List, Optional, Protocol, Tuple

from . import ps100_registers as R
from ..config import PS100Config

log = logging.getLogger(__name__)


class Transport(Protocol):
    def write_register(self, address: int, value: int) -> None: ...
    def read_register(self, address: int) -> int: ...


class ModbusTransport:
    """Real drive. Values are sent as signed 16-bit (function code 6/3)."""

    def __init__(self, cfg: PS100Config):
        import minimalmodbus
        import serial

        parity = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN,
                  "O": serial.PARITY_ODD}[cfg.parity.upper()]
        inst = minimalmodbus.Instrument(cfg.port, cfg.slave_id,
                                        mode=minimalmodbus.MODE_RTU)
        inst.serial.baudrate = cfg.baudrate
        inst.serial.bytesize = 8
        inst.serial.parity = parity
        inst.serial.stopbits = cfg.stopbits
        inst.serial.timeout = cfg.timeout_s
        inst.clear_buffers_before_each_transaction = True
        inst.close_port_after_each_call = False
        self._inst = inst
        self._offset = cfg.address_offset
        self._lock = threading.Lock()

    def write_register(self, address: int, value: int) -> None:
        with self._lock:
            self._inst.write_register(address + self._offset, int(value),
                                      number_of_decimals=0, functioncode=6,
                                      signed=True)

    def read_register(self, address: int) -> int:
        with self._lock:
            return self._inst.read_register(address + self._offset,
                                            number_of_decimals=0,
                                            functioncode=3, signed=False)


class SimTransport:
    """
    Fake drive. Records writes; on a 0 -> 1 edge of 0x011F it "moves" for
    `move_time_s` and then toggles bit 0 of 0x1010, matching the manual's
    "observe a change in bit 0" description.
    """

    def __init__(self, move_time_s: float = 0.05):
        self.registers: Dict[int, int] = {R.REG_OUTPUT_STATUS: 0,
                                          R.REG_VIRTUAL_INPUT_CONTROL: 0}
        self.writes: List[Tuple[int, int]] = []
        self.move_time_s = move_time_s
        self._done_at: Optional[float] = None

    def write_register(self, address: int, value: int) -> None:
        if not -32768 <= value <= 65535:
            raise ValueError(f"value {value} does not fit a 16-bit register")
        prev = self.registers.get(address, 0)
        self.registers[address] = value
        self.writes.append((address, value))
        log.debug("[SIM] write 0x%04X <- %d", address, value)
        if (address == R.REG_VIRTUAL_INPUT_CONTROL and prev == R.CONTROL_IDLE
                and value == R.CONTROL_POSITION_TRIGGER):
            self._done_at = time.monotonic() + self.move_time_s

    def read_register(self, address: int) -> int:
        if self._done_at is not None and time.monotonic() >= self._done_at:
            self.registers[R.REG_OUTPUT_STATUS] ^= 1 << R.POSITION_COMPLETE_BIT
            self._done_at = None
        return self.registers.get(address, 0)


class PS100:
    """High-level PS100 driver: relative moves with speed, plus stop."""

    def __init__(self, transport: Transport, cfg: PS100Config):
        self.t = transport
        self.cfg = cfg

    @classmethod
    def from_config(cls, cfg: PS100Config, dry_run: bool) -> "PS100":
        transport: Transport = SimTransport() if dry_run else ModbusTransport(cfg)
        if dry_run:
            log.info("PS100 in DRY RUN mode (simulated drive, no serial port)")
        return cls(transport, cfg)

    # -- low level -------------------------------------------------------
    def _check_i16(self, name: str, value: int) -> int:
        value = int(value)
        if not -32768 <= value <= 32767:
            raise ValueError(f"{name}={value} out of signed 16-bit range")
        return value

    def status_bit(self) -> int:
        return (self.t.read_register(R.REG_OUTPUT_STATUS) >> R.POSITION_COMPLETE_BIT) & 1

    # -- motion ------------------------------------------------------------
    def load_move(self, turns: int, pulses: int, rpm: int) -> None:
        """Write the three move registers (does not start motion)."""
        if rpm <= 0:
            raise ValueError("speed (0x0204) must be strictly positive")
        if turns and pulses and (turns > 0) != (pulses > 0):
            raise ValueError("turns and pulses must have the same sign")
        if abs(pulses) >= R.PULSES_PER_REV:
            raise ValueError(f"pulses {pulses} must be < {R.PULSES_PER_REV} per turn")
        self.t.write_register(R.REG_POSITION_TURNS, self._check_i16("turns", turns))
        self.t.write_register(R.REG_POSITION_PULSES, self._check_i16("pulses", pulses))
        self.t.write_register(R.REG_POSITION_SPEED, self._check_i16("rpm", rpm))

    def trigger(self) -> None:
        """Rising edge on virtual DI1 (internal-position start)."""
        self.t.write_register(R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_IDLE)
        self.t.write_register(R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_POSITION_TRIGGER)

    def stop(self) -> None:
        """Software stop (needs P3-39 = 27). NOT an emergency stop."""
        try:
            self.t.write_register(R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_POSITION_STOP)
        finally:
            self.t.write_register(R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_IDLE)

    def move(self, turns: int, pulses: int, rpm: int,
             wait: bool = True, est_time_s: Optional[float] = None) -> bool:
        """
        Load + trigger a relative move. If `wait`, block until complete
        (per completion_mode) and return True on completion, False on timeout.
        """
        self.load_move(turns, pulses, rpm)
        mode = self.cfg.completion_mode
        before = self.status_bit() if mode == "change" else None
        log.info("PS100 move: turns=%d pulses=%d rpm=%d", turns, pulses, rpm)
        self.trigger()
        if not wait:
            return True
        return self.wait_complete(before, est_time_s)

    def wait_complete(self, before: Optional[int], est_time_s: Optional[float]) -> bool:
        mode = self.cfg.completion_mode
        if mode == "time":
            time.sleep((est_time_s or 0.5) * 1.2 + 0.05)
            return True
        deadline = time.monotonic() + self.cfg.completion_timeout_s
        while time.monotonic() < deadline:
            bit = self.status_bit()
            if (mode == "change" and bit != before) or (mode == "set" and bit == 1):
                return True
            time.sleep(0.01)
        log.warning("PS100 move did not report completion within %.1fs",
                    self.cfg.completion_timeout_s)
        return False
