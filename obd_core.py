"""ELM327 protocol layer, response parsing and OBD-II decoding.

Kept free of any GUI code so it can be tested against a simulated adapter.

Two response formats are handled:

* CAN (ISO 15765-4, protocols 6-9, e.g. every Ford sold from ~2008):
  headers are switched on (ATH1) so each line starts with the CAN ID of
  the module that answered (7E8 = engine, 7E9 = transmission, …).
  Multi-frame ISO-TP messages are reassembled per module.
* Everything else (ISO 9141-2 / KWP2000 K-line, e.g. older Subarus):
  headers stay off and every line is treated as one message, exactly like
  the original scanner did.
"""

from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import serial

from dtc_codes import describe_dtc

HEX_RE = re.compile(r"^[0-9A-F]+$")
NO_DATA_WORDS = ("NO DATA", "UNABLE TO CONNECT", "BUS INIT", "STOPPED", "ERROR",
                 "CAN ERROR", "BUFFER FULL", "BUS BUSY", "FB ERROR", "DATA ERROR")

PROTOCOL_NAMES = {
    "1": "SAE J1850 PWM",
    "2": "SAE J1850 VPW",
    "3": "ISO 9141-2",
    "4": "ISO 14230-4 KWP (5 baud init)",
    "5": "ISO 14230-4 KWP (fast init)",
    "6": "ISO 15765-4 CAN (11 bit, 500 kbaud)",
    "7": "ISO 15765-4 CAN (29 bit, 500 kbaud)",
    "8": "ISO 15765-4 CAN (11 bit, 250 kbaud)",
    "9": "ISO 15765-4 CAN (29 bit, 250 kbaud)",
    "A": "SAE J1939 CAN",
}
CAN_PROTOCOLS = {"6", "7", "8", "9"}
CAN_29BIT = {"7", "9"}

# ATSP argument per vehicle profile. "A6" = start with CAN 11-bit/500k and
# fall back to the automatic search if the car does not answer on it.
VEHICLE_PROFILES = {
    "Auto detect": "0",
    "Ford (CAN)": "A6",
    "Subaru / K-line": "0",
}

NRC_NAMES = {
    0x10: "general reject",
    0x11: "service not supported",
    0x12: "sub-function not supported",
    0x13: "incorrect message length",
    0x22: "conditions not correct (e.g. engine running)",
    0x31: "request out of range",
    0x33: "security access denied",
    0x72: "general programming failure",
    0x78: "response pending",
    0x7E: "sub-function not supported in active session",
    0x7F: "service not supported in active session",
}

# OBD-II response addresses (ISO 15765-4).
ECU_NAMES = {
    "7E8": "Engine (ECM/PCM)",
    "7E9": "Transmission (TCM)",
    "7EA": "ECU #3 (7EA)",
    "7EB": "ECU #4 (7EB)",
    "7EC": "Hybrid / ECU #5 (7EC)",
    "7ED": "ECU #6 (7ED)",
    "7EE": "ECU #7 (7EE)",
    "7EF": "ECU #8 (7EF)",
    "18DAF110": "Engine (ECM/PCM)",
    "18DAF118": "Transmission (TCM)",
    "": "ECU",
}

VIN_MAKES = {
    "Ford": ("1FA", "1FB", "1FC", "1FD", "1FM", "1FT", "2FA", "2FM", "2FT", "3FA", "3FM", "3FT",
             "MAJ", "MNB", "NM0", "WF0", "WF1", "6FP", "LVS", "1LN", "2LM", "3LN", "5LM", "5LT",
             "1ME", "2ME", "3ME", "4M2"),
    "Subaru": ("JF1", "JF2", "JF3", "4S3", "4S4", "4S6"),
}


def ecu_name(address: str) -> str:
    return ECU_NAMES.get(address.upper(), f"Module {address}")


def make_from_vin(vin: str) -> str:
    for make, prefixes in VIN_MAKES.items():
        if vin[:3].upper() in prefixes:
            return make
    return ""


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

@dataclass
class EcuMessage:
    address: str          # "7E8", "18DAF110", or "" when headers are off
    data: list[int]       # payload, starting with the service byte

    @property
    def negative(self) -> bool:
        return len(self.data) >= 3 and self.data[0] == 0x7F

    @property
    def nrc(self) -> int | None:
        return self.data[2] if self.negative else None


def describe_nrc(code: int | None) -> str:
    if code is None:
        return ""
    return NRC_NAMES.get(code, f"negative response 0x{code:02X}")


def _hex_lines(raw: str) -> list[str]:
    """Upper-case, space-free lines with adapter status text removed."""
    text = raw.upper().replace("\r", "\n").replace("SEARCHING...", "")
    lines = []
    for line in text.splitlines():
        compact = line.strip().replace(" ", "")
        if not compact:
            continue
        if any(word.replace(" ", "") in compact for word in NO_DATA_WORDS):
            continue
        lines.append(compact)
    return lines


def to_bytes(line: str) -> list[int]:
    return [int(line[i:i + 2], 16) for i in range(0, len(line), 2)]


def parse_can_response(raw: str, header_len: int = 3) -> list[EcuMessage]:
    """Parse ATH1 output and reassemble ISO-TP frames per responding module."""
    buffers: dict[str, list[int]] = {}
    expected: dict[str, int] = {}
    done: list[EcuMessage] = []   # in arrival order; multi-frame messages fill in later

    for line in _hex_lines(raw):
        if not HEX_RE.fullmatch(line) or len(line) < header_len + 2:
            continue
        if (len(line) - header_len) % 2:
            continue
        address = line[:header_len]
        frame = to_bytes(line[header_len:])
        pci_type = frame[0] >> 4

        if pci_type == 0:  # single frame
            length = frame[0] & 0x0F
            if length:
                done.append(EcuMessage(address, frame[1:1 + length]))
        elif pci_type == 1 and len(frame) >= 2:  # first frame
            expected[address] = ((frame[0] & 0x0F) << 8) | frame[1]
            buffers[address] = frame[2:]
            done.append(EcuMessage(address, buffers[address]))
        elif pci_type == 2 and address in buffers:  # consecutive frame
            buffers[address].extend(frame[1:])

    for message in done:
        if message.address in expected and message.data is buffers.get(message.address):
            del message.data[expected[message.address]:]
    return done


def parse_plain_response(raw: str) -> list[EcuMessage]:
    """Parse ATH0 output. Each line is one message; legacy 'n:' CAN lines are joined."""
    messages: list[EcuMessage] = []
    multi: list[int] = []
    total: int | None = None

    for line in _hex_lines(raw):
        if ":" in line:
            multi.extend(to_bytes(re.sub(r"[^0-9A-F]", "", line.split(":", 1)[1])))
            continue
        line = re.sub(r"[^0-9A-F]", "", line)
        if len(line) == 3 and not multi:  # ISO-TP byte count line, e.g. "014"
            total = int(line, 16)
            continue
        if line and len(line) % 2 == 0:
            messages.append(EcuMessage("", to_bytes(line)))

    if multi:
        messages.append(EcuMessage("", multi[:total] if total else multi))
    return messages


# ---------------------------------------------------------------------------
# PID catalog and value decoding
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PIDDef:
    pid: int
    name: str
    unit: str = ""
    priority: int = 2  # 1 = quick profile, 2 = extended, 3 = obscure/slow
    length: int = 1    # data bytes in the response (needed for multi-PID requests)


def pct(a: int) -> float:
    return a * 100.0 / 255.0


def trim(a: int) -> float:
    return (a - 128) * 100.0 / 128.0


_P = PIDDef
PID_CATALOG: dict[int, PIDDef] = {p.pid: p for p in [
    _P(0x01, "Monitor status since DTCs cleared", "", 1, 4),
    _P(0x02, "Freeze-frame DTC", "", 2, 2),
    _P(0x03, "Fuel system status", "", 1, 2),
    _P(0x04, "Calculated engine load", "%", 1),
    _P(0x05, "Engine coolant temperature", "°C", 1),
    _P(0x06, "Short-term fuel trim Bank 1", "%", 1),
    _P(0x07, "Long-term fuel trim Bank 1", "%", 1),
    _P(0x08, "Short-term fuel trim Bank 2", "%", 2),
    _P(0x09, "Long-term fuel trim Bank 2", "%", 2),
    _P(0x0A, "Fuel pressure", "kPa", 2),
    _P(0x0B, "Intake manifold absolute pressure", "kPa", 1),
    _P(0x0C, "Engine RPM", "rpm", 1, 2),
    _P(0x0D, "Vehicle speed", "km/h", 1),
    _P(0x0E, "Ignition timing advance", "°", 1),
    _P(0x0F, "Intake air temperature", "°C", 1),
    _P(0x10, "MAF airflow", "g/s", 1, 2),
    _P(0x11, "Throttle position", "%", 1),
    _P(0x12, "Commanded secondary air status", "", 2),
    _P(0x13, "Oxygen sensors present", "", 2),
    _P(0x14, "O2 sensor B1S1", "", 1, 2),
    _P(0x15, "O2 sensor B1S2", "", 1, 2),
    _P(0x16, "O2 sensor B1S3", "", 2, 2),
    _P(0x17, "O2 sensor B1S4", "", 2, 2),
    _P(0x18, "O2 sensor B2S1", "", 2, 2),
    _P(0x19, "O2 sensor B2S2", "", 2, 2),
    _P(0x1A, "O2 sensor B2S3", "", 2, 2),
    _P(0x1B, "O2 sensor B2S4", "", 2, 2),
    _P(0x1C, "OBD standard", "", 2),
    _P(0x1D, "Oxygen sensors present (4 banks)", "", 3),
    _P(0x1E, "Auxiliary input status", "", 3),
    _P(0x1F, "Engine run time", "s", 2, 2),
    _P(0x21, "Distance travelled with MIL on", "km", 2, 2),
    _P(0x22, "Fuel rail pressure relative to manifold", "kPa", 2, 2),
    _P(0x23, "Fuel rail gauge pressure", "kPa", 2, 2),
    _P(0x24, "Wideband O2 B1S1 equivalence ratio / voltage", "", 2, 4),
    _P(0x25, "Wideband O2 B1S2 equivalence ratio / voltage", "", 2, 4),
    _P(0x26, "Wideband O2 B1S3 equivalence ratio / voltage", "", 3, 4),
    _P(0x27, "Wideband O2 B1S4 equivalence ratio / voltage", "", 3, 4),
    _P(0x28, "Wideband O2 B2S1 equivalence ratio / voltage", "", 3, 4),
    _P(0x29, "Wideband O2 B2S2 equivalence ratio / voltage", "", 3, 4),
    _P(0x2A, "Wideband O2 B2S3 equivalence ratio / voltage", "", 3, 4),
    _P(0x2B, "Wideband O2 B2S4 equivalence ratio / voltage", "", 3, 4),
    _P(0x2C, "Commanded EGR", "%", 3),
    _P(0x2D, "EGR error", "%", 3),
    _P(0x2E, "Commanded evaporative purge", "%", 2),
    _P(0x2F, "Fuel tank level", "%", 2),
    _P(0x30, "Warm-ups since codes cleared", "", 2),
    _P(0x31, "Distance since codes cleared", "km", 2, 2),
    _P(0x32, "Evap system vapour pressure", "Pa", 3, 2),
    _P(0x33, "Absolute barometric pressure", "kPa", 1),
    _P(0x34, "Wideband O2 B1S1 equivalence ratio / current", "", 2, 4),
    _P(0x35, "Wideband O2 B1S2 equivalence ratio / current", "", 2, 4),
    _P(0x36, "Wideband O2 B1S3 equivalence ratio / current", "", 3, 4),
    _P(0x37, "Wideband O2 B1S4 equivalence ratio / current", "", 3, 4),
    _P(0x38, "Wideband O2 B2S1 equivalence ratio / current", "", 3, 4),
    _P(0x39, "Wideband O2 B2S2 equivalence ratio / current", "", 3, 4),
    _P(0x3A, "Wideband O2 B2S3 equivalence ratio / current", "", 3, 4),
    _P(0x3B, "Wideband O2 B2S4 equivalence ratio / current", "", 3, 4),
    _P(0x3C, "Catalyst temperature B1S1", "°C", 3, 2),
    _P(0x3D, "Catalyst temperature B2S1", "°C", 3, 2),
    _P(0x3E, "Catalyst temperature B1S2", "°C", 3, 2),
    _P(0x3F, "Catalyst temperature B2S2", "°C", 3, 2),
    _P(0x41, "Monitor status this drive cycle", "", 3, 4),
    _P(0x42, "Control module voltage", "V", 1, 2),
    _P(0x43, "Absolute engine load", "%", 2, 2),
    _P(0x44, "Commanded equivalence ratio", "λ", 2, 2),
    _P(0x45, "Relative throttle position", "%", 2),
    _P(0x46, "Ambient air temperature", "°C", 2),
    _P(0x47, "Absolute throttle position B", "%", 2),
    _P(0x48, "Absolute throttle position C", "%", 2),
    _P(0x49, "Accelerator pedal position D", "%", 2),
    _P(0x4A, "Accelerator pedal position E", "%", 2),
    _P(0x4B, "Accelerator pedal position F", "%", 2),
    _P(0x4C, "Commanded throttle actuator", "%", 2),
    _P(0x4D, "Time run with MIL on", "min", 2, 2),
    _P(0x4E, "Time since trouble codes cleared", "min", 2, 2),
    _P(0x4F, "Maximum values for fuel/air sensors", "", 3, 4),
    _P(0x50, "Maximum MAF airflow", "g/s", 3, 4),
    _P(0x51, "Fuel type", "", 2),
    _P(0x52, "Ethanol fuel percentage", "%", 3),
    _P(0x53, "Absolute evap system vapour pressure", "kPa", 3, 2),
    _P(0x54, "Evap system vapour pressure", "Pa", 3, 2),
    _P(0x55, "Short-term secondary O2 trim B1/B3", "", 3, 2),
    _P(0x56, "Long-term secondary O2 trim B1/B3", "", 3, 2),
    _P(0x57, "Short-term secondary O2 trim B2/B4", "", 3, 2),
    _P(0x58, "Long-term secondary O2 trim B2/B4", "", 3, 2),
    _P(0x59, "Fuel rail absolute pressure", "kPa", 3, 2),
    _P(0x5A, "Relative accelerator pedal position", "%", 2),
    _P(0x5B, "Hybrid battery remaining life", "%", 3),
    _P(0x5C, "Engine oil temperature", "°C", 2),
    _P(0x5D, "Fuel injection timing", "°", 3, 2),
    _P(0x5E, "Engine fuel rate", "L/h", 3, 2),
    _P(0x61, "Driver demanded engine torque", "%", 3),
    _P(0x62, "Actual engine torque", "%", 3),
    _P(0x63, "Engine reference torque", "Nm", 3, 2),
    _P(0x64, "Engine percent torque data", "%", 3, 5),
    _P(0x66, "MAF sensor A/B", "g/s", 3, 5),
    _P(0x67, "Engine coolant temperature sensors A/B", "°C", 3, 3),
    _P(0x8E, "Engine friction percent torque", "%", 3),
    _P(0xA6, "Odometer", "km", 2, 4),
]}

QUICK_PID_IDS = [pid for pid, definition in PID_CATALOG.items() if definition.priority == 1]
SUPPORT_PIDS = (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0)


def decode_dtc_pair(a: int, b: int) -> str:
    if a == 0 and b == 0:
        return ""
    prefix = "PCBU"[(a >> 6) & 0x03]
    digit = (a >> 4) & 0x03
    remaining = ((a & 0x0F) << 8) | b
    return f"{prefix}{digit}{remaining:03X}"


def decode_monitor_status(data: list[int]) -> str:
    """Short form, also written to the CSV log (the drive-log analyzer parses it)."""
    if len(data) < 4:
        return "Insufficient data"
    a, b = data[0], data[1]
    mil = bool(a & 0x80)
    count = a & 0x7F
    ignition = "compression" if b & 0x08 else "spark"
    incomplete = sum(1 for _, available, complete in readiness_monitors(data) if available and not complete)
    return f"MIL {'ON' if mil else 'OFF'}; {count} DTC(s); {ignition}-ignition; {incomplete} monitor(s) not ready"


SPARK_MONITORS = ("Catalyst", "Heated catalyst", "Evaporative system", "Secondary air system",
                  "A/C refrigerant", "Oxygen sensor", "Oxygen sensor heater", "EGR / VVT system")
COMPRESSION_MONITORS = ("NMHC catalyst", "NOx / SCR aftertreatment", "", "Boost pressure",
                        "", "Exhaust gas sensor", "PM filter", "EGR / VVT system")


def readiness_monitors(data: list[int]) -> list[tuple[str, bool, bool]]:
    """Return (monitor, supported, complete) for PID 01 / 41 data."""
    if len(data) < 4:
        return []
    _, b, c, d = data[:4]
    result = []
    for index, name in enumerate(("Misfire", "Fuel system", "Comprehensive components")):
        result.append((name, bool(b & (1 << index)), not (b & (1 << (index + 4)))))
    names = COMPRESSION_MONITORS if b & 0x08 else SPARK_MONITORS
    for bit, name in enumerate(names):
        if name:
            result.append((name, bool(c & (1 << bit)), not (d & (1 << bit))))
    return result


def decode_fuel_status(data: list[int]) -> str:
    mapping = {
        0x01: "Open loop: insufficient temperature",
        0x02: "Closed loop: O2 feedback",
        0x04: "Open loop: engine load/deceleration",
        0x08: "Open loop: system failure",
        0x10: "Closed loop with O2 fault",
    }
    values = [mapping.get(v, f"0x{v:02X}") for v in data[:2] if v]
    return " / ".join(values) if values else "No status"


def decode_obd_standard(a: int) -> str:
    standards = {
        1: "OBD-II (CARB)", 2: "OBD (EPA)", 3: "OBD + OBD-II", 4: "OBD-I",
        5: "Not OBD compliant", 6: "EOBD", 7: "EOBD + OBD-II", 8: "EOBD + OBD",
        9: "EOBD + OBD + OBD-II", 10: "JOBD", 11: "JOBD + OBD-II", 12: "JOBD + EOBD",
        13: "JOBD + EOBD + OBD-II",
    }
    return standards.get(a, f"Standard code {a}")


def decode_fuel_type(a: int) -> str:
    types = {
        0: "Not available", 1: "Gasoline", 2: "Methanol", 3: "Ethanol",
        4: "Diesel", 5: "LPG", 6: "CNG", 7: "Propane", 8: "Electric",
        9: "Bifuel gasoline", 10: "Bifuel methanol", 11: "Bifuel ethanol",
        12: "Bifuel LPG", 13: "Bifuel CNG", 14: "Bifuel propane",
        15: "Bifuel electricity", 16: "Bifuel electric/combustion",
        17: "Hybrid gasoline", 18: "Hybrid ethanol", 19: "Hybrid diesel",
        20: "Hybrid electric", 21: "Hybrid mixed", 22: "Hybrid regenerative",
    }
    return types.get(a, f"Fuel type code {a}")


def decode_pid_value(pid: int, data: list[int]) -> tuple[str, float | None]:
    if not data:
        return "No data", None

    a = data[0]
    b = data[1] if len(data) > 1 else 0
    c = data[2] if len(data) > 2 else 0
    d = data[3] if len(data) > 3 else 0
    ab = a * 256 + b

    if pid in (0x01, 0x41):
        return decode_monitor_status(data), None
    if pid == 0x02:
        return decode_dtc_pair(a, b) or "No DTC", None
    if pid == 0x03:
        return decode_fuel_status(data), None
    if pid in (0x04, 0x11, 0x2C, 0x2E, 0x2F, 0x45, 0x47, 0x48,
               0x49, 0x4A, 0x4B, 0x4C, 0x52, 0x5A, 0x5B):
        value = pct(a)
        return f"{value:.1f}", value
    if pid in (0x05, 0x0F, 0x46, 0x5C):
        value = a - 40
        return f"{value:.0f}", float(value)
    if pid in (0x06, 0x07, 0x08, 0x09, 0x2D):
        value = trim(a)
        return f"{value:.1f}", value
    if pid == 0x0A:
        value = a * 3
        return f"{value:.0f}", float(value)
    if pid in (0x0B, 0x33):
        return f"{a}", float(a)
    if pid == 0x0C:
        value = ab / 4.0
        return f"{value:.0f}", value
    if pid == 0x0D:
        return f"{a}", float(a)
    if pid == 0x0E:
        value = a / 2.0 - 64
        return f"{value:.1f}", value
    if pid == 0x10:
        value = ab / 100.0
        return f"{value:.2f}", value
    if pid == 0x12:
        mapping = {1: "Upstream", 2: "Downstream", 4: "Outside atmosphere", 8: "Pump commanded off"}
        return mapping.get(a, f"Status 0x{a:02X}"), None
    if pid in (0x13, 0x1D):
        sensors = [str(i + 1) for i in range(8) if a & (1 << i)]
        return "Sensors " + ", ".join(sensors) if sensors else "None reported", None
    if 0x14 <= pid <= 0x1B:
        voltage = a / 200.0
        if b == 0xFF:
            return f"{voltage:.3f} V; trim n/a", voltage
        return f"{voltage:.3f} V; trim {trim(b):.1f} %", voltage
    if pid == 0x1C:
        return decode_obd_standard(a), None
    if pid == 0x1E:
        return "PTO active" if a & 1 else "PTO inactive", None
    if pid in (0x1F, 0x21, 0x31, 0x4D, 0x4E, 0x63):
        return f"{ab}", float(ab)
    if pid == 0x22:
        value = ab * 0.079
        return f"{value:.1f}", value
    if pid in (0x23, 0x59):
        value = ab * 10.0
        return f"{value:.0f}", value
    if 0x24 <= pid <= 0x2B:
        eq = ab * 2.0 / 65536.0
        voltage = (c * 256 + d) * 8.0 / 65536.0
        return f"λ {eq:.3f}; {voltage:.3f} V", eq
    if pid == 0x30:
        return f"{a}", float(a)
    if pid in (0x32, 0x54):
        raw = ab if ab < 32768 else ab - 65536
        value = raw / 4.0 if pid == 0x32 else float(raw)
        return f"{value:.1f}" if pid == 0x32 else f"{raw}", value
    if 0x34 <= pid <= 0x3B:
        eq = ab * 2.0 / 65536.0
        current = (c * 256 + d) / 256.0 - 128.0
        return f"λ {eq:.3f}; {current:.3f} mA", eq
    if 0x3C <= pid <= 0x3F:
        value = ab / 10.0 - 40.0
        return f"{value:.1f}", value
    if pid == 0x42:
        value = ab / 1000.0
        return f"{value:.3f}", value
    if pid == 0x43:
        value = ab * 100.0 / 255.0
        return f"{value:.1f}", value
    if pid == 0x44:
        value = ab / 32768.0
        return f"{value:.3f}", value
    if pid == 0x4F:
        return f"Eq max {a}; O2 voltage max {b}; O2 current max {c}; MAP max {d * 10}", None
    if pid == 0x50:
        value = a * 10.0
        return f"{value:.0f}", value
    if pid == 0x51:
        return decode_fuel_type(a), None
    if pid == 0x53:
        value = ab / 200.0
        return f"{value:.3f}", value
    if 0x55 <= pid <= 0x58:
        return f"Bank A {trim(a):.1f} %; Bank B {trim(b):.1f} %", None
    if pid == 0x5D:
        value = ab / 128.0 - 210.0
        return f"{value:.2f}", value
    if pid == 0x5E:
        value = ab / 20.0
        return f"{value:.2f}", value
    if pid in (0x61, 0x62):
        value = a - 125.0
        return f"{value:.0f}", value
    if pid == 0x66 and len(data) >= 5:
        values = [(data[i] * 256 + data[i + 1]) / 32.0 for i in (1, 3)]
        present = [values[i] for i in range(2) if a & (1 << i)] or values[:1]
        return " / ".join(f"{v:.2f}" for v in present), present[0]
    if pid == 0x67 and len(data) >= 3:
        present = [data[i + 1] - 40 for i in range(2) if a & (1 << i)] or [b - 40]
        return " / ".join(f"{v:.0f}" for v in present), float(present[0])
    if pid == 0x8E:
        return f"{a - 125:.0f}", float(a - 125)
    if pid == 0xA6 and len(data) >= 4:
        value = int.from_bytes(bytes(data[:4]), "big") / 10.0
        return f"{value:.1f}", value
    if pid == 0x64 and len(data) >= 5:
        values = [(x - 125) for x in data[:5]]
        return " / ".join(str(x) for x in values), None

    return " ".join(f"{byte:02X}" for byte in data), None


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------

class ELM327:
    def __init__(self) -> None:
        self.serial: serial.Serial | None = None
        self.lock = threading.Lock()
        self.protocol = ""         # ATDPN digit, e.g. "6"
        self.version = ""
        self.on_raw: Callable[[str, str], None] | None = None

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    @property
    def is_can(self) -> bool:
        return self.protocol in CAN_PROTOCOLS

    @property
    def header_len(self) -> int:
        return 8 if self.protocol in CAN_29BIT else 3

    @property
    def protocol_name(self) -> str:
        return PROTOCOL_NAMES.get(self.protocol, f"Protocol {self.protocol or '?'}")

    def open(self, port: str, baud: int) -> None:
        self.disconnect()
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.12, write_timeout=1)
        time.sleep(0.3)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def connect(self, port: str, baud: int, protocol: str = "0") -> list[str]:
        """Initialise the adapter and the vehicle bus. Returns the init transcript."""
        self.open(port, baud)
        return self.initialise(protocol)

    def initialise(self, protocol: str = "0") -> list[str]:
        replies: list[str] = []

        def run(command: str, timeout: float = 2.0) -> str:
            response = self.send(command, timeout)
            replies.append(f"{command} -> {response}")
            return response

        match = re.search(r"ELM327\s*v?[\d.]+", run("ATZ", 4.0), re.IGNORECASE)
        self.version = match.group(0) if match else "ELM327 (version unknown)"
        for command in ("ATE0", "ATL0", "ATS0", "ATH0", "ATAT1", f"ATSP{protocol}"):
            run(command)

        if "4100" not in run("0100", 8.0).replace(" ", "").upper() and protocol != "0":
            run("ATSP0")
            run("0100", 8.0)

        number = run("ATDPN").strip().upper().lstrip("A")
        self.protocol = number[-1:] if number else ""
        run("ATDP")
        if self.is_can:
            run("ATH1")
            run("ATCAF1")
        return replies

    def disconnect(self) -> None:
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.protocol = ""

    def send(self, command: str, timeout: float = 2.5) -> str:
        if not self.connected or self.serial is None:
            raise RuntimeError("ELM327 is not connected.")

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write((command.strip() + "\r").encode("ascii"))
            self.serial.flush()

            deadline = time.monotonic() + timeout
            buffer = bytearray()
            while time.monotonic() < deadline:
                chunk = self.serial.read(self.serial.in_waiting or 1)
                if chunk:
                    buffer.extend(chunk)
                    if b">" in buffer:
                        break
                else:
                    time.sleep(0.01)

            response = buffer.decode("ascii", errors="replace").replace(">", "").strip()

        if self.on_raw:
            self.on_raw(command, response)
        return response

    def request(self, command: str, timeout: float = 3.0) -> list[EcuMessage]:
        raw = self.send(command, timeout)
        if self.is_can:
            messages = parse_can_response(raw, self.header_len)
        else:
            messages = parse_plain_response(raw)
        # "Response pending" is only an interim answer.
        return [m for m in messages if m.nrc != 0x78]


# ---------------------------------------------------------------------------
# Generic OBD-II services
# ---------------------------------------------------------------------------

def pid_data(messages: list[EcuMessage], mode: int, pid: int,
             address: str | None = None) -> list[int] | None:
    """Data bytes of the first mode/PID answer (from `address` when given)."""
    expected = mode + 0x40
    skip = 3 if mode == 0x02 else 2  # mode 02 echoes the frame number after the PID
    for message in messages:
        if address is not None and message.address and message.address != address:
            continue
        data = message.data
        for index in range(len(data) - 1):
            if data[index] == expected and data[index + 1] == pid:
                return data[index + skip:]
    return None


MAX_PIDS_PER_REQUEST = 6  # ISO 15765-4 limit for one mode 01 request


def split_multi_pid(data: list[int], requested: list[int]) -> dict[int, list[int]] | None:
    """Split a 41 <pid> <data> <pid> <data> … answer. None if it cannot be parsed safely."""
    if not data or data[0] != 0x41:
        return None
    result: dict[int, list[int]] = {}
    index = 1
    while index < len(data):
        pid = data[index]
        if pid not in requested or pid in result or pid not in PID_CATALOG:
            return None
        length = PID_CATALOG[pid].length
        if index + 1 + length > len(data):
            return None
        result[pid] = data[index + 1:index + 1 + length]
        index += 1 + length
    return result


class PIDReader:
    """Reads mode 01 PIDs, several per request on CAN (much faster), one by one otherwise."""

    def __init__(self, elm: ELM327, address: str | None) -> None:
        self.elm = elm
        self.address = address
        self.batch = elm.is_can
        self.failures = 0

    def _single(self, pid: int) -> list[int] | None:
        return pid_data(self.elm.request(f"01{pid:02X}", 3.0), 0x01, pid, self.address)

    def read(self, pids: list[int]):
        """Yield (pid, data or None) for the requested PIDs."""
        size = MAX_PIDS_PER_REQUEST if self.batch else 1
        for start in range(0, len(pids), size):
            chunk = pids[start:start + size]
            parsed = None
            if self.batch and len(chunk) > 1:
                command = "01" + "".join(f"{pid:02X}" for pid in chunk)
                for message in self.elm.request(command, 3.0):
                    if self.address is None or not message.address or message.address == self.address:
                        parsed = split_multi_pid(message.data, chunk)
                        if parsed is not None:
                            break
                if parsed is None:
                    self.failures += 1
                    if self.failures >= 3:  # this ECU does not like multi-PID requests
                        self.batch = False
            if parsed is None:
                for pid in chunk:
                    yield pid, self._single(pid)
            else:
                for pid in chunk:
                    yield pid, parsed.get(pid)


def discover_supported_pids(elm: ELM327) -> tuple[str, dict[str, set[int]]]:
    """Return (primary engine address, {address: supported mode 01 PIDs})."""
    supported: dict[str, set[int]] = {}
    for base in SUPPORT_PIDS:
        command = f"01{base:02X}"
        messages = elm.request(command, 4.0)
        got_any = False
        for message in messages:
            data = pid_data([message], 0x01, base)
            if not data or len(data) < 4:
                continue
            got_any = True
            mask = int.from_bytes(bytes(data[:4]), "big")
            pids = supported.setdefault(message.address, set())
            for bit in range(32):
                if mask & (1 << (31 - bit)):
                    pids.add(base + bit + 1)
        if not got_any:
            if base == 0:
                raise RuntimeError("ECU did not return supported PID data.")
            break
        # Continue only while some ECU says the next range exists.
        if not any(base + 0x20 in pids for pids in supported.values()):
            break

    primary = "7E8" if "7E8" in supported else "18DAF110" if "18DAF110" in supported else \
        max(supported, key=lambda key: len(supported[key]))
    return primary, supported


@dataclass
class DTC:
    code: str
    status: str             # "Stored", "Pending", "Permanent", "Active", …
    module: str             # human readable module name
    description: str = ""
    detail: str = ""        # e.g. failure type / raw status for enhanced reads


def decode_dtc_payload(payload: list[int], is_can: bool) -> list[str]:
    """Decode the bytes after the 43/47/4A service byte."""
    # On CAN the first byte is the number of DTCs. Guard against ECUs that omit it.
    if is_can and payload and len(payload) == 1 + 2 * payload[0]:
        payload = payload[1:]
    elif is_can and len(payload) % 2 == 1:
        payload = payload[1:]
    codes = []
    for i in range(0, len(payload) - 1, 2):
        code = decode_dtc_pair(payload[i], payload[i + 1])
        if code and code not in codes:
            codes.append(code)
    return codes


DTC_SERVICES = ((0x03, "Stored"), (0x07, "Pending"), (0x0A, "Permanent"))


def read_generic_dtcs(elm: ELM327, make: str = "") -> tuple[list[DTC], list[str]]:
    """Read stored, pending and permanent codes from every responding ECU."""
    found: list[DTC] = []
    notes: list[str] = []
    for service, status in DTC_SERVICES:
        messages = elm.request(f"{service:02X}", 6.0)
        answered = False
        for message in messages:
            if message.negative:
                if message.data[1] == service:
                    answered = True
                    notes.append(f"{status} codes: {ecu_name(message.address)} – {describe_nrc(message.nrc)}")
                continue
            if not message.data or message.data[0] != service + 0x40:
                continue
            answered = True
            for code in decode_dtc_payload(message.data[1:], elm.is_can):
                if not any(d.code == code and d.status == status and d.module == ecu_name(message.address)
                           for d in found):
                    found.append(DTC(code, status, ecu_name(message.address), describe_dtc(code, make)))
        if not answered:
            notes.append(f"{status} codes: no answer (service {service:02X} may be unsupported)")
    return found, notes


def read_monitor_status(elm: ELM327) -> dict[str, list[int]]:
    """PID 01 per ECU: {address: [A, B, C, D]}."""
    result = {}
    for message in elm.request("0101", 4.0):
        data = pid_data([message], 0x01, 0x01)
        if data and len(data) >= 4:
            result[message.address] = data[:4]
    return result


def format_readiness(status: dict[str, list[int]]) -> str:
    if not status:
        return "Monitor status (PID 01) not available."
    lines = []
    for address, data in status.items():
        mil = "ON" if data[0] & 0x80 else "OFF"
        lines.append(f"{ecu_name(address)}: MIL {mil}, {data[0] & 0x7F} confirmed emission DTC(s)")
        kind = "compression" if data[1] & 0x08 else "spark"
        lines.append(f"  Readiness monitors ({kind} ignition):")
        for name, available, complete in readiness_monitors(data):
            if available:
                lines.append(f"    {'READY    ' if complete else 'NOT READY'}  {name}")
    return "\n".join(lines)


def clear_dtcs(elm: ELM327) -> str:
    messages = elm.request("04", 8.0)
    lines = []
    for message in messages:
        if message.data[:1] == [0x44]:
            lines.append(f"{ecu_name(message.address)}: codes cleared")
        elif message.negative:
            lines.append(f"{ecu_name(message.address)}: refused – {describe_nrc(message.nrc)}")
    return "\n".join(lines) or "No confirmation received. Ignition ON, engine OFF is usually required."


def read_info_string(elm: ELM327, pid: int) -> str:
    """Mode 09 ASCII info (02 = VIN, 04 = calibration ID, 0A = ECU name)."""
    messages = elm.request(f"09{pid:02X}", 5.0)
    chunks: list[tuple[int, list[int]]] = []
    for message in messages:
        data = message.data
        if len(data) < 3 or data[0] != 0x49 or data[1] != pid:
            continue
        # CAN: 49 PID count <ascii…>.  K-line: one line per 4 bytes: 49 PID seq b1 b2 b3 b4.
        chunks.append((data[2] if not elm.is_can else 0, data[3:]))
        if elm.is_can:
            break
    raw = [byte for _, part in sorted(chunks, key=lambda item: item[0]) for byte in part]
    text = "".join(chr(b) for b in raw if 32 <= b < 127)
    return text.strip()


def read_vehicle_info(elm: ELM327) -> dict[str, str]:
    info = {"vin": read_info_string(elm, 0x02)}
    info["make"] = make_from_vin(info["vin"])
    if elm.is_can:
        info["calibration"] = read_info_string(elm, 0x04)
        info["ecu_name"] = read_info_string(elm, 0x0A)
    return info


def read_freeze_frame(elm: ELM327, pids: list[int], address: str | None,
                      make: str = "") -> list[str]:
    """Freeze frame 00. Stops early when no freeze frame is stored."""
    lines = ["FREEZE FRAME (frame 00)"]
    trigger = pid_data(elm.request("020200", 3.5), 0x02, 0x02, address)
    if trigger is not None and len(trigger) >= 2:
        code = decode_dtc_pair(trigger[0], trigger[1])
        if not code:
            return lines + ["No freeze frame stored."]
        lines.append(f"Triggered by {code} – {describe_dtc(code, make)}")
    lines.append("")

    found = False
    for pid in pids:
        if pid in (0x01, 0x02) or pid not in PID_CATALOG:
            continue
        data = pid_data(elm.request(f"02{pid:02X}00", 3.5), 0x02, pid, address)
        if data:
            value, _ = decode_pid_value(pid, data)
            definition = PID_CATALOG[pid]
            lines.append(f"02 {pid:02X}  {definition.name}: {value} {definition.unit}".rstrip())
            found = True
    if not found:
        lines.append("No freeze-frame values returned.")
    return lines


# ---------------------------------------------------------------------------
# Live data interpretation
# ---------------------------------------------------------------------------

def derive_values(values: dict[int, tuple[str, float | None]], make: str = "") -> dict[str, object]:
    numeric = {pid: value for pid, (_, value) in values.items() if value is not None}
    lines = ["LIVE DERIVED VALUES / BASIC INTERPRETATION", ""]
    csv_values: dict[str, object] = {
        "Derived boost relative to atmosphere (kPa)": "",
        "Derived boost relative to atmosphere (bar)": "",
        "Combined fuel trim Bank 1 (%)": "",
    }

    map_kpa = numeric.get(0x0B)
    baro_kpa = numeric.get(0x33)
    if map_kpa is not None and baro_kpa is not None:
        relative_kpa = map_kpa - baro_kpa
        relative_bar = relative_kpa / 100.0
        csv_values["Derived boost relative to atmosphere (kPa)"] = f"{relative_kpa:.1f}"
        csv_values["Derived boost relative to atmosphere (bar)"] = f"{relative_bar:.3f}"
        if relative_kpa >= 0:
            lines.append(f"Boost: {relative_kpa:.1f} kPa / {relative_bar:.3f} bar above atmosphere")
        else:
            lines.append(f"Intake vacuum: {-relative_kpa:.1f} kPa below atmosphere")
    elif map_kpa is not None:
        lines.append("Boost cannot be calculated accurately because BARO PID 01 33 is unavailable.")

    for bank, (st_pid, lt_pid) in enumerate(((0x06, 0x07), (0x08, 0x09)), start=1):
        stft = numeric.get(st_pid)
        ltft = numeric.get(lt_pid)
        if stft is None or ltft is None:
            continue
        combined = stft + ltft
        if bank == 1:
            csv_values["Combined fuel trim Bank 1 (%)"] = f"{combined:.1f}"
        lines.append(f"Combined Bank {bank} correction: {combined:+.1f} % (STFT {stft:+.1f}, LTFT {ltft:+.1f})")
        if combined > 20:
            lines.append("  Strong positive correction: substantial lean tendency or under-reported airflow.")
        elif combined > 10:
            lines.append("  Moderate positive correction: inspect intake leaks, MAF scaling and fuel delivery.")
        elif combined < -15:
            lines.append("  Strong negative correction: rich tendency.")
        else:
            lines.append("  Fuel correction is within a broadly plausible range.")

    coolant = numeric.get(0x05)
    if coolant is not None:
        if coolant < 70:
            lines.append(f"Coolant: {coolant:.0f} °C — engine may not yet be fully warm.")
        elif coolant > 105:
            lines.append(f"Coolant: {coolant:.0f} °C — unusually hot; verify cooling system.")
        else:
            lines.append(f"Coolant: {coolant:.0f} °C")

    voltage = numeric.get(0x42)
    rpm = numeric.get(0x0C)
    if voltage is not None:
        if rpm and rpm > 400:
            note = "normal charging range" if 13.2 <= voltage <= 14.8 else "check charging system"
        else:
            note = "reasonable engine-off voltage" if 11.8 <= voltage <= 12.8 else "check battery voltage"
        lines.append(f"Module voltage: {voltage:.2f} V — {note}")

    maf = numeric.get(0x10)
    if maf is not None and rpm is not None and rpm > 0:
        lines.append(f"MAF: {maf:.2f} g/s at {rpm:.0f} rpm")

    lines.append("")
    if make == "Subaru":
        lines.append("Interpret these as clues, not automatic proof. Generic OBD-II cannot expose Subaru "
                     "knock correction, IAM, wastegate duty or cylinder-specific misfire counters.")
    else:
        lines.append("Interpret these as clues, not automatic proof. Generic OBD-II only exposes "
                     "emission-related values; manufacturer data needs enhanced diagnostics.")

    return {"text": "\n".join(lines), "csv": csv_values}
