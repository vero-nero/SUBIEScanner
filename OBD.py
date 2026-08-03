#!/usr/bin/env python3
"""
ELM327 Universal OBD-II Scanner / Logger v2
Target setup:
- Windows
- USB ELM327 / FTDI
- ISO 9141-2
- Typical port: COM4
- baud 38400 IMPORTANT!!!

Dependency:
    py -m pip install pyserial
"""

from __future__ import annotations

import csv
import math
import queue
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


HEX_RE = re.compile(r"^[0-9A-F]+$")
NO_DATA_WORDS = ("NO DATA", "UNABLE TO CONNECT", "BUS INIT", "STOPPED", "ERROR")
LOG_DIR = Path(__file__).resolve().parent / "logs"


@dataclass(frozen=True)
class PIDDef:
    pid: int
    name: str
    unit: str = ""
    priority: int = 2  # 1 = quick profile, 2 = extended, 3 = obscure/slow


def pct(a: int) -> float:
    return a * 100.0 / 255.0


def trim(a: int) -> float:
    return (a - 128) * 100.0 / 128.0


PID_CATALOG: dict[int, PIDDef] = {
    0x01: PIDDef(0x01, "Monitor status since DTCs cleared", "", 1),
    0x02: PIDDef(0x02, "Freeze-frame DTC", "", 2),
    0x03: PIDDef(0x03, "Fuel system status", "", 1),
    0x04: PIDDef(0x04, "Calculated engine load", "%", 1),
    0x05: PIDDef(0x05, "Engine coolant temperature", "°C", 1),
    0x06: PIDDef(0x06, "Short-term fuel trim Bank 1", "%", 1),
    0x07: PIDDef(0x07, "Long-term fuel trim Bank 1", "%", 1),
    0x08: PIDDef(0x08, "Short-term fuel trim Bank 2", "%", 2),
    0x09: PIDDef(0x09, "Long-term fuel trim Bank 2", "%", 2),
    0x0A: PIDDef(0x0A, "Fuel pressure", "kPa", 2),
    0x0B: PIDDef(0x0B, "Intake manifold absolute pressure", "kPa", 1),
    0x0C: PIDDef(0x0C, "Engine RPM", "rpm", 1),
    0x0D: PIDDef(0x0D, "Vehicle speed", "km/h", 1),
    0x0E: PIDDef(0x0E, "Ignition timing advance", "°", 1),
    0x0F: PIDDef(0x0F, "Intake air temperature", "°C", 1),
    0x10: PIDDef(0x10, "MAF airflow", "g/s", 1),
    0x11: PIDDef(0x11, "Throttle position", "%", 1),
    0x12: PIDDef(0x12, "Commanded secondary air status", "", 2),
    0x13: PIDDef(0x13, "Oxygen sensors present", "", 2),
    0x14: PIDDef(0x14, "O2 sensor B1S1", "", 1),
    0x15: PIDDef(0x15, "O2 sensor B1S2", "", 1),
    0x16: PIDDef(0x16, "O2 sensor B1S3", "", 2),
    0x17: PIDDef(0x17, "O2 sensor B1S4", "", 2),
    0x18: PIDDef(0x18, "O2 sensor B2S1", "", 2),
    0x19: PIDDef(0x19, "O2 sensor B2S2", "", 2),
    0x1A: PIDDef(0x1A, "O2 sensor B2S3", "", 2),
    0x1B: PIDDef(0x1B, "O2 sensor B2S4", "", 2),
    0x1C: PIDDef(0x1C, "OBD standard", "", 2),
    0x1D: PIDDef(0x1D, "Oxygen sensors present (4 banks)", "", 3),
    0x1E: PIDDef(0x1E, "Auxiliary input status", "", 3),
    0x1F: PIDDef(0x1F, "Engine run time", "s", 2),
    0x21: PIDDef(0x21, "Distance travelled with MIL on", "km", 2),
    0x22: PIDDef(0x22, "Fuel rail pressure relative to manifold", "kPa", 2),
    0x23: PIDDef(0x23, "Fuel rail gauge pressure", "kPa", 2),
    0x24: PIDDef(0x24, "Wideband O2 B1S1 equivalence ratio / voltage", "", 2),
    0x25: PIDDef(0x25, "Wideband O2 B1S2 equivalence ratio / voltage", "", 2),
    0x26: PIDDef(0x26, "Wideband O2 B1S3 equivalence ratio / voltage", "", 3),
    0x27: PIDDef(0x27, "Wideband O2 B1S4 equivalence ratio / voltage", "", 3),
    0x28: PIDDef(0x28, "Wideband O2 B2S1 equivalence ratio / voltage", "", 3),
    0x29: PIDDef(0x29, "Wideband O2 B2S2 equivalence ratio / voltage", "", 3),
    0x2A: PIDDef(0x2A, "Wideband O2 B2S3 equivalence ratio / voltage", "", 3),
    0x2B: PIDDef(0x2B, "Wideband O2 B2S4 equivalence ratio / voltage", "", 3),
    0x2C: PIDDef(0x2C, "Commanded EGR", "%", 3),
    0x2D: PIDDef(0x2D, "EGR error", "%", 3),
    0x2E: PIDDef(0x2E, "Commanded evaporative purge", "%", 2),
    0x2F: PIDDef(0x2F, "Fuel tank level", "%", 2),
    0x30: PIDDef(0x30, "Warm-ups since codes cleared", "", 2),
    0x31: PIDDef(0x31, "Distance since codes cleared", "km", 2),
    0x32: PIDDef(0x32, "Evap system vapour pressure", "Pa", 3),
    0x33: PIDDef(0x33, "Absolute barometric pressure", "kPa", 1),
    0x34: PIDDef(0x34, "Wideband O2 B1S1 equivalence ratio / current", "", 2),
    0x35: PIDDef(0x35, "Wideband O2 B1S2 equivalence ratio / current", "", 2),
    0x36: PIDDef(0x36, "Wideband O2 B1S3 equivalence ratio / current", "", 3),
    0x37: PIDDef(0x37, "Wideband O2 B1S4 equivalence ratio / current", "", 3),
    0x38: PIDDef(0x38, "Wideband O2 B2S1 equivalence ratio / current", "", 3),
    0x39: PIDDef(0x39, "Wideband O2 B2S2 equivalence ratio / current", "", 3),
    0x3A: PIDDef(0x3A, "Wideband O2 B2S3 equivalence ratio / current", "", 3),
    0x3B: PIDDef(0x3B, "Wideband O2 B2S4 equivalence ratio / current", "", 3),
    0x3C: PIDDef(0x3C, "Catalyst temperature B1S1", "°C", 3),
    0x3D: PIDDef(0x3D, "Catalyst temperature B2S1", "°C", 3),
    0x3E: PIDDef(0x3E, "Catalyst temperature B1S2", "°C", 3),
    0x3F: PIDDef(0x3F, "Catalyst temperature B2S2", "°C", 3),
    0x41: PIDDef(0x41, "Monitor status this drive cycle", "", 3),
    0x42: PIDDef(0x42, "Control module voltage", "V", 1),
    0x43: PIDDef(0x43, "Absolute engine load", "%", 2),
    0x44: PIDDef(0x44, "Commanded equivalence ratio", "λ", 2),
    0x45: PIDDef(0x45, "Relative throttle position", "%", 2),
    0x46: PIDDef(0x46, "Ambient air temperature", "°C", 2),
    0x47: PIDDef(0x47, "Absolute throttle position B", "%", 2),
    0x48: PIDDef(0x48, "Absolute throttle position C", "%", 2),
    0x49: PIDDef(0x49, "Accelerator pedal position D", "%", 2),
    0x4A: PIDDef(0x4A, "Accelerator pedal position E", "%", 2),
    0x4B: PIDDef(0x4B, "Accelerator pedal position F", "%", 2),
    0x4C: PIDDef(0x4C, "Commanded throttle actuator", "%", 2),
    0x4D: PIDDef(0x4D, "Time run with MIL on", "min", 2),
    0x4E: PIDDef(0x4E, "Time since trouble codes cleared", "min", 2),
    0x4F: PIDDef(0x4F, "Maximum values for fuel/air sensors", "", 3),
    0x50: PIDDef(0x50, "Maximum MAF airflow", "g/s", 3),
    0x51: PIDDef(0x51, "Fuel type", "", 2),
    0x52: PIDDef(0x52, "Ethanol fuel percentage", "%", 3),
    0x53: PIDDef(0x53, "Absolute evap system vapour pressure", "kPa", 3),
    0x54: PIDDef(0x54, "Evap system vapour pressure", "Pa", 3),
    0x55: PIDDef(0x55, "Short-term secondary O2 trim B1/B3", "", 3),
    0x56: PIDDef(0x56, "Long-term secondary O2 trim B1/B3", "", 3),
    0x57: PIDDef(0x57, "Short-term secondary O2 trim B2/B4", "", 3),
    0x58: PIDDef(0x58, "Long-term secondary O2 trim B2/B4", "", 3),
    0x59: PIDDef(0x59, "Fuel rail absolute pressure", "kPa", 3),
    0x5A: PIDDef(0x5A, "Relative accelerator pedal position", "%", 2),
    0x5B: PIDDef(0x5B, "Hybrid battery remaining life", "%", 3),
    0x5C: PIDDef(0x5C, "Engine oil temperature", "°C", 2),
    0x5D: PIDDef(0x5D, "Fuel injection timing", "°", 3),
    0x5E: PIDDef(0x5E, "Engine fuel rate", "L/h", 3),
    0x61: PIDDef(0x61, "Driver demanded engine torque", "%", 3),
    0x62: PIDDef(0x62, "Actual engine torque", "%", 3),
    0x63: PIDDef(0x63, "Engine reference torque", "Nm", 3),
    0x64: PIDDef(0x64, "Engine percent torque data", "%", 3),
}


QUICK_PID_IDS = [pid for pid, definition in PID_CATALOG.items() if definition.priority == 1]


def clean_lines(raw: str, command: str) -> list[str]:
    command = command.replace(" ", "").upper()
    text = raw.upper().replace("\r", "\n").replace("SEARCHING...", "")
    output: list[str] = []

    for line in text.splitlines():
        compact = line.strip().replace(" ", "")
        if not compact or compact == command:
            continue
        if any(word.replace(" ", "") in compact for word in NO_DATA_WORDS):
            continue
        if ":" in compact:
            compact = compact.split(":", 1)[1]
        compact = re.sub(r"[^0-9A-F]", "", compact)
        if compact and len(compact) % 2 == 0 and HEX_RE.fullmatch(compact):
            output.append(compact)

    return output


def to_bytes(line: str) -> list[int]:
    return [int(line[i:i + 2], 16) for i in range(0, len(line), 2)]


def find_mode_pid_response(raw: str, mode: int, pid: int, command: str) -> list[int] | None:
    expected_mode = mode + 0x40
    for line in clean_lines(raw, command):
        data = to_bytes(line)
        for index in range(len(data) - 1):
            if data[index] == expected_mode and data[index + 1] == pid:
                return data[index + 2:]
    return None


def decode_dtc_pair(a: int, b: int) -> str:
    if a == 0 and b == 0:
        return ""
    prefix = "PCBU"[(a >> 6) & 0x03]
    digit = (a >> 4) & 0x03
    remaining = ((a & 0x0F) << 8) | b
    return f"{prefix}{digit}{remaining:03X}"


def decode_dtcs(raw: str, mode: int, command: str) -> list[str]:
    expected = mode + 0x40
    found: list[str] = []

    for line in clean_lines(raw, command):
        data = to_bytes(line)
        try:
            payload = data[data.index(expected) + 1:]
        except ValueError:
            continue

        for i in range(0, len(payload) - 1, 2):
            code = decode_dtc_pair(payload[i], payload[i + 1])
            if code and code not in found:
                found.append(code)

    return found


def decode_monitor_status(data: list[int]) -> str:
    if len(data) < 4:
        return "Insufficient data"
    a, b, c, d = data[:4]
    mil = bool(a & 0x80)
    count = a & 0x7F
    ignition = "compression" if b & 0x08 else "spark"
    return f"MIL {'ON' if mil else 'OFF'}; {count} DTC(s); {ignition}-ignition monitor set"


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
        1: "OBD-II (CARB)",
        2: "OBD (EPA)",
        3: "OBD + OBD-II",
        4: "OBD-I",
        5: "Not OBD compliant",
        6: "EOBD",
        7: "EOBD + OBD-II",
        8: "EOBD + OBD",
        9: "EOBD + OBD + OBD-II",
        10: "JOBD",
        11: "JOBD + OBD-II",
        12: "JOBD + EOBD",
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
    abcd = (a << 24) | (b << 16) | (c << 8) | d

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
        tr = trim(b)
        return f"{voltage:.3f} V; trim {tr:.1f} %", voltage
    if pid == 0x1C:
        return decode_obd_standard(a), None
    if pid == 0x1E:
        return "PTO active" if a & 1 else "PTO inactive", None
    if pid == 0x1F:
        return f"{ab}", float(ab)
    if pid in (0x21, 0x31):
        return f"{ab}", float(ab)
    if pid == 0x22:
        value = ab * 0.079
        return f"{value:.1f}", value
    if pid == 0x23:
        value = ab * 10.0
        return f"{value:.0f}", value
    if 0x24 <= pid <= 0x2B:
        eq = ab * 2.0 / 65536.0
        voltage = (c * 256 + d) * 8.0 / 65536.0
        return f"λ {eq:.3f}; {voltage:.3f} V", eq
    if pid == 0x30:
        return f"{a}", float(a)
    if pid == 0x32:
        raw = ab if ab < 32768 else ab - 65536
        value = raw / 4.0
        return f"{value:.1f}", value
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
    if pid in (0x4D, 0x4E):
        return f"{ab}", float(ab)
    if pid == 0x4F:
        return f"Eq max {a}; O2 voltage max {b}; O2 current max {c}; MAP max {d}", None
    if pid == 0x50:
        value = a * 10.0
        return f"{value:.0f}", value
    if pid == 0x51:
        return decode_fuel_type(a), None
    if pid == 0x53:
        value = ab / 200.0
        return f"{value:.3f}", value
    if pid == 0x54:
        raw = ab if ab < 32768 else ab - 65536
        return f"{raw}", float(raw)
    if 0x55 <= pid <= 0x58:
        return f"Bank A {trim(a):.1f} %; Bank B {trim(b):.1f} %", None
    if pid == 0x59:
        value = abcd * 10.0
        return f"{value:.0f}", value
    if pid == 0x5D:
        value = ab / 128.0 - 210.0
        return f"{value:.2f}", value
    if pid == 0x5E:
        value = ab / 20.0
        return f"{value:.2f}", value
    if pid in (0x61, 0x62):
        value = a - 125.0
        return f"{value:.0f}", value
    if pid == 0x63:
        return f"{ab}", float(ab)
    if pid == 0x64 and len(data) >= 5:
        values = [(x - 125) for x in data[:5]]
        return " / ".join(str(x) for x in values), None

    return " ".join(f"{byte:02X}" for byte in data), None


def discover_supported_pids(elm: "ELM327") -> set[int]:
    supported: set[int] = set()
    for base in (0x00, 0x20, 0x40, 0x60):
        command = f"01{base:02X}"
        raw = elm.send(command, 4.0)
        data = find_mode_pid_response(raw, 0x01, base, command)
        if not data or len(data) < 4:
            if base == 0:
                raise RuntimeError(f"ECU did not return supported PID data: {raw}")
            break

        mask = int.from_bytes(bytes(data[:4]), "big")
        for bit in range(32):
            if mask & (1 << (31 - bit)):
                supported.add(base + bit + 1)

        if not (mask & 1):
            break

    return supported


class ELM327:
    def __init__(self) -> None:
        self.serial: serial.Serial | None = None
        self.lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    def connect(self, port: str, baud: int) -> tuple[str, list[str]]:
        self.disconnect()
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.12, write_timeout=1)
        time.sleep(0.3)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        replies: list[str] = []
        sequence = [
            ("ATZ", 4.0),
            ("ATE0", 2.0),
            ("ATL0", 2.0),
            ("ATS0", 2.0),
            ("ATH0", 2.0),
            ("ATSP0", 2.0),
            ("0100", 8.0),
            ("ATDP", 2.0),
        ]
        for command, timeout in sequence:
            response = self.send(command, timeout)
            replies.append(f"{command} -> {response}")

        protocol = replies[-1].split("->", 1)[-1].strip()
        return protocol, replies

    def disconnect(self) -> None:
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None

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

            return buffer.decode("ascii", errors="replace").replace(">", "").strip()


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ELM327 OBD-II Scanner / Logger v2")
        self.root.geometry("1120x760")
        self.root.minsize(900, 620)

        self.elm = ELM327()
        self.queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.monitoring = False
        self.supported: set[int] = set()
        self.current_values: dict[int, tuple[str, float | None]] = {}
        self.log_path: Path | None = None
        self.csv_handle = None
        self.csv_writer = None

        self.port_var = tk.StringVar(value="COM4")
        self.baud_var = tk.StringVar(value="38400")
        self.profile_var = tk.StringVar(value="Quick diagnostic")
        self.interval_var = tk.StringVar(value="0.3")
        self.raw_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Disconnected")
        self.summary_var = tk.StringVar(value="No ECU data")

        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def build_ui(self) -> None:
        connection = ttk.Frame(self.root, padding=10)
        connection.pack(fill="x")

        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(0, 4))
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=12)
        self.port_box.grid(row=0, column=1, padx=(0, 8))

        ttk.Label(connection, text="Baud").grid(row=0, column=2, padx=(0, 4))
        ttk.Combobox(connection, textvariable=self.baud_var,
                     values=("38400", "9600", "115200"),
                     state="readonly", width=10).grid(row=0, column=3, padx=(0, 8))

        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=4, padx=3)
        ttk.Button(connection, text="Connect + discover", command=self.connect).grid(row=0, column=5, padx=3)
        ttk.Button(connection, text="Disconnect", command=self.disconnect).grid(row=0, column=6, padx=3)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=7, padx=(15, 0), sticky="w")
        connection.columnconfigure(7, weight=1)

        toolbar = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        toolbar.pack(fill="x")

        ttk.Button(toolbar, text="Read DTCs", command=self.read_dtcs).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Read freeze frame", command=self.read_freeze).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Clear DTCs", command=self.clear_dtcs).pack(side="left", padx=3)

        ttk.Separator(toolbar, orient="vertical").pack(side="left", fill="y", padx=8)

        ttk.Label(toolbar, text="Profile").pack(side="left")
        ttk.Combobox(toolbar, textvariable=self.profile_var,
                     values=("Quick diagnostic", "All supported PIDs"),
                     state="readonly", width=19).pack(side="left", padx=4)

        ttk.Label(toolbar, text="Pause between PIDs").pack(side="left", padx=(8, 0))
        ttk.Entry(toolbar, textvariable=self.interval_var, width=6).pack(side="left", padx=4)
        ttk.Label(toolbar, text="s").pack(side="left")

        ttk.Button(toolbar, text="Start logging", command=self.start_monitoring).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Stop", command=self.stop_monitoring).pack(side="left", padx=4)
        ttk.Checkbutton(toolbar, text="Raw responses", variable=self.raw_var).pack(side="left", padx=8)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        live_tab = ttk.Frame(notebook)
        diagnosis_tab = ttk.Frame(notebook)
        dtc_tab = ttk.Frame(notebook)
        console_tab = ttk.Frame(notebook)
        notebook.add(live_tab, text="Live data")
        notebook.add(diagnosis_tab, text="Derived / diagnosis")
        notebook.add(dtc_tab, text="DTC / freeze frame")
        notebook.add(console_tab, text="Console")

        self.live_tree = ttk.Treeview(
            live_tab,
            columns=("pid", "name", "value", "unit", "support"),
            show="headings",
        )
        for key, title, width in (
            ("pid", "PID", 70), ("name", "Parameter", 430),
            ("value", "Value", 280), ("unit", "Unit", 90),
            ("support", "Supported", 90),
        ):
            self.live_tree.heading(key, text=title)
            self.live_tree.column(key, width=width)
        self.live_tree.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(live_tab, orient="vertical", command=self.live_tree.yview)
        self.live_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.place(relx=1.0, rely=0, relheight=1.0, anchor="ne")

        self.diagnosis_text = tk.Text(diagnosis_tab, wrap="word", font=("Consolas", 10))
        self.diagnosis_text.pack(fill="both", expand=True)

        self.dtc_text = tk.Text(dtc_tab, wrap="word", font=("Consolas", 10))
        self.dtc_text.pack(fill="both", expand=True)

        self.console_text = tk.Text(console_tab, wrap="word", font=("Consolas", 9))
        self.console_text.pack(fill="both", expand=True)

        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bottom.pack(fill="x")
        ttk.Label(bottom, textvariable=self.summary_var).pack(side="left")
        self.log_label = ttk.Label(bottom, text="Log: not selected")
        self.log_label.pack(side="right", padx=4)
        ttk.Button(bottom, text="Choose CSV", command=self.choose_log).pack(side="right")

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def background(self, func) -> None:
        threading.Thread(target=func, daemon=True).start()

    def require_connection(self) -> bool:
        if not self.elm.connected:
            messagebox.showerror("Not connected", "Connect to the ELM327 first.")
            return False
        return True

    def connect(self) -> None:
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Invalid baud", "Choose a valid baud rate.")
            return
        if not port:
            messagebox.showerror("No port", "Choose a COM port.")
            return

        self.status_var.set("Connecting…")

        def worker() -> None:
            try:
                protocol, replies = self.elm.connect(port, baud)
                for line in replies:
                    self.queue.put(("console", line))
                supported = discover_supported_pids(self.elm)
                self.queue.put(("connected", (protocol, supported)))
            except Exception as exc:
                self.elm.disconnect()
                self.queue.put(("error", f"Connection/discovery failed:\n{exc}"))

        self.background(worker)

    def disconnect(self) -> None:
        self.stop_monitoring()
        self.elm.disconnect()
        self.supported.clear()
        self.status_var.set("Disconnected")
        self.summary_var.set("No ECU data")

    def populate_tree(self) -> None:
        for item in self.live_tree.get_children():
            self.live_tree.delete(item)

        for pid in sorted(PID_CATALOG):
            definition = PID_CATALOG[pid]
            supported = "Yes" if pid in self.supported else "No"
            self.live_tree.insert(
                "", "end", iid=f"{pid:02X}",
                values=(f"01 {pid:02X}", definition.name, "—", definition.unit, supported),
            )

    def selected_pids(self) -> list[int]:
        if self.profile_var.get() == "All supported PIDs":
            return [pid for pid in sorted(self.supported) if pid in PID_CATALOG and pid not in (0x20, 0x40, 0x60)]
        return [pid for pid in QUICK_PID_IDS if pid in self.supported]

    def choose_log(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="Choose CSV log",
            defaultextension=".csv",
            initialdir=LOG_DIR,
            initialfile=f"obd_log_{datetime.now():%Y%m%d_%H%M%S}.csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if filename:
            self.log_path = Path(filename)
            self.log_label.config(text=f"Log: {self.log_path}")

    def start_monitoring(self) -> None:
        if not self.require_connection() or self.monitoring:
            return

        pids = self.selected_pids()
        if not pids:
            messagebox.showerror("No supported PIDs", "No PIDs are available for this profile.")
            return

        try:
            pause = max(0.0, float(self.interval_var.get().replace(",", ".")))
        except ValueError:
            messagebox.showerror("Invalid pause", "Use a number such as 0.3.")
            return

        if self.log_path is None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            self.log_path = LOG_DIR / f"obd_log_{datetime.now():%Y%m%d_%H%M%S}.csv"

        self.csv_handle = self.log_path.open("w", newline="", encoding="utf-8-sig")
        fields = ["timestamp"] + [PID_CATALOG[p].name for p in pids] + [
            "Derived boost relative to atmosphere (kPa)",
            "Derived boost relative to atmosphere (bar)",
            "Combined fuel trim Bank 1 (%)",
        ]
        self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=fields, delimiter=";")
        self.csv_writer.writeheader()
        self.log_label.config(text=f"Log: {self.log_path}")

        self.stop_event.clear()
        self.monitoring = True
        self.status_var.set(f"Logging {len(pids)} PIDs")

        def worker() -> None:
            try:
                while not self.stop_event.is_set():
                    row: dict[str, object] = {"timestamp": datetime.now().isoformat(timespec="milliseconds")}
                    cycle_values: dict[int, tuple[str, float | None]] = {}

                    for pid in pids:
                        if self.stop_event.is_set():
                            break
                        command = f"01{pid:02X}"
                        raw = self.elm.send(command, 3.0)
                        data = find_mode_pid_response(raw, 0x01, pid, command)

                        if data is None:
                            text, numeric = "No data", None
                        else:
                            text, numeric = decode_pid_value(pid, data)

                        cycle_values[pid] = (text, numeric)
                        row[PID_CATALOG[pid].name] = text
                        self.queue.put(("live", (pid, text)))

                        if self.raw_var.get():
                            self.queue.put(("console", f"{command} -> {raw}"))

                        if pause:
                            self.stop_event.wait(pause)

                    derived = derive_values(cycle_values)
                    row.update(derived["csv"])
                    self.csv_writer.writerow(row)
                    self.csv_handle.flush()
                    self.queue.put(("diagnosis", derived["text"]))

            except Exception as exc:
                self.queue.put(("error", f"Logging stopped:\n{exc}"))
            finally:
                self.monitoring = False
                if self.csv_handle:
                    self.csv_handle.close()
                self.csv_handle = None
                self.csv_writer = None
                self.queue.put(("stopped", None))

        self.background(worker)

    def stop_monitoring(self) -> None:
        self.stop_event.set()

    def read_dtcs(self) -> None:
        if not self.require_connection():
            return

        def worker() -> None:
            try:
                result = []
                for mode, title in ((0x03, "STORED"), (0x07, "PENDING"), (0x0A, "PERMANENT")):
                    command = f"{mode:02X}"
                    raw = self.elm.send(command, 5)
                    codes = decode_dtcs(raw, mode, command)
                    result.append(f"{title} CODES\n" + ("\n".join(codes) if codes else "None or unsupported"))
                    if self.raw_var.get():
                        self.queue.put(("console", f"{command} -> {raw}"))
                self.queue.put(("dtc", "\n\n".join(result)))
            except Exception as exc:
                self.queue.put(("error", f"Could not read DTCs:\n{exc}"))

        self.background(worker)

    def read_freeze(self) -> None:
        if not self.require_connection():
            return

        pids = [pid for pid in sorted(self.supported) if pid in PID_CATALOG and pid not in (0x20, 0x40, 0x60)]

        def worker() -> None:
            try:
                lines = ["FREEZE FRAME (frame 00)", ""]
                found = False
                for pid in pids:
                    command = f"02{pid:02X}00"
                    raw = self.elm.send(command, 3.5)
                    data = find_mode_pid_response(raw, 0x02, pid, command)
                    if data is not None:
                        value, _ = decode_pid_value(pid, data)
                        lines.append(f"02 {pid:02X}  {PID_CATALOG[pid].name}: {value} {PID_CATALOG[pid].unit}".rstrip())
                        found = True
                    if self.raw_var.get():
                        self.queue.put(("console", f"{command} -> {raw}"))
                if not found:
                    lines.append("No freeze-frame values returned.")
                self.queue.put(("dtc", "\n".join(lines)))
            except Exception as exc:
                self.queue.put(("error", f"Freeze-frame read failed:\n{exc}"))

        self.background(worker)

    def clear_dtcs(self) -> None:
        if not self.require_connection():
            return
        if not messagebox.askyesno(
            "Clear DTCs",
            "This clears stored codes, readiness information and freeze-frame data.\n\nContinue?",
        ):
            return

        def worker() -> None:
            try:
                raw = self.elm.send("04", 6)
                self.queue.put(("dtc", f"CLEAR DTC RESPONSE\n\n{raw or 'No response'}"))
                self.queue.put(("console", f"04 -> {raw}"))
            except Exception as exc:
                self.queue.put(("error", f"Could not clear DTCs:\n{exc}"))

        self.background(worker)

    def process_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()

                if kind == "connected":
                    protocol, supported = payload
                    self.supported = set(supported)
                    self.populate_tree()
                    known = sum(1 for pid in supported if pid in PID_CATALOG)
                    self.status_var.set(f"Connected: {protocol}")
                    self.summary_var.set(f"ECU reports {len(supported)} supported PIDs; {known} decoded")
                elif kind == "live":
                    pid, value = payload
                    iid = f"{pid:02X}"
                    if self.live_tree.exists(iid):
                        old = self.live_tree.item(iid, "values")
                        self.live_tree.item(iid, values=(old[0], old[1], value, old[3], old[4]))
                elif kind == "diagnosis":
                    self.diagnosis_text.delete("1.0", "end")
                    self.diagnosis_text.insert("1.0", str(payload))
                elif kind == "dtc":
                    self.dtc_text.delete("1.0", "end")
                    self.dtc_text.insert("1.0", str(payload))
                elif kind == "console":
                    self.console_text.insert("end", str(payload) + "\n")
                    self.console_text.see("end")
                elif kind == "stopped":
                    if self.elm.connected:
                        self.status_var.set("Connected")
                elif kind == "error":
                    self.status_var.set("Error")
                    messagebox.showerror("Error", str(payload))
        except queue.Empty:
            pass

        self.root.after(100, self.process_queue)

    def close(self) -> None:
        self.stop_event.set()
        self.elm.disconnect()
        if self.csv_handle:
            self.csv_handle.close()
        self.root.destroy()


def derive_values(values: dict[int, tuple[str, float | None]]) -> dict[str, object]:
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

    stft = numeric.get(0x06)
    ltft = numeric.get(0x07)
    if stft is not None and ltft is not None:
        combined = stft + ltft
        csv_values["Combined fuel trim Bank 1 (%)"] = f"{combined:.1f}"
        lines.append(f"Combined Bank 1 correction: {combined:+.1f} % (STFT {stft:+.1f}, LTFT {ltft:+.1f})")
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
    lines.append("Interpret these as clues, not automatic proof. Generic OBD-II cannot expose Subaru knock correction, IAM, wastegate duty or cylinder-specific misfire counters on this ECU.")

    return {"text": "\n".join(lines), "csv": csv_values}


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
