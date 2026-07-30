#!/usr/bin/env python3
"""
ELM327 Subaru OBD-II Scanner and Live Logger
Tested design target:
- Windows
- USB ELM327 with FTDI
- ISO 9141-2
- Typical port: COM4

Required package:
    py -m pip install pyserial
"""

from __future__ import annotations

import csv
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


PROMPT = ">"
HEX_RE = re.compile(r"^[0-9A-F]+$")


@dataclass(frozen=True)
class PID:
    command: str
    name: str
    unit: str


LIVE_PIDS = [
    PID("010C", "Engine RPM", "rpm"),
    PID("0104", "Calculated Load", "%"),
    PID("0105", "Coolant Temperature", "°C"),
    PID("010F", "Intake Air Temperature", "°C"),
    PID("0110", "MAF Airflow", "g/s"),
    PID("0106", "Short Fuel Trim Bank 1", "%"),
    PID("0107", "Long Fuel Trim Bank 1", "%"),
    PID("010B", "Intake Manifold Pressure", "kPa"),
    PID("010E", "Ignition Timing Advance", "°"),
    PID("0111", "Throttle Position", "%"),
    PID("010D", "Vehicle Speed", "km/h"),
    PID("0103", "Fuel System Status", ""),
    PID("0114", "O2 Sensor B1S1 Voltage", "V"),
    PID("0115", "O2 Sensor B1S2 Voltage", "V"),
]

FREEZE_PIDS = [
    PID("0202", "Freeze-frame DTC", ""),
    PID("020C", "Engine RPM", "rpm"),
    PID("0204", "Calculated Load", "%"),
    PID("0205", "Coolant Temperature", "°C"),
    PID("020F", "Intake Air Temperature", "°C"),
    PID("0210", "MAF Airflow", "g/s"),
    PID("0206", "Short Fuel Trim Bank 1", "%"),
    PID("0207", "Long Fuel Trim Bank 1", "%"),
    PID("020B", "Intake Manifold Pressure", "kPa"),
    PID("020E", "Ignition Timing Advance", "°"),
    PID("0211", "Throttle Position", "%"),
    PID("020D", "Vehicle Speed", "km/h"),
]


def clean_lines(raw: str, command: str) -> list[str]:
    command = command.replace(" ", "").upper()
    text = raw.upper().replace("\r", "\n").replace("SEARCHING...", "")
    result: list[str] = []

    for line in text.splitlines():
        line = line.strip().replace(" ", "")
        if not line or line in {command, "OK", "NODATA", "STOPPED", "?", "UNABLETOCONNECT"}:
            continue

        # Remove CAN-style headers or line counters only when obvious.
        if ":" in line:
            line = line.split(":", 1)[1]

        line = re.sub(r"[^0-9A-F]", "", line)
        if line and len(line) % 2 == 0 and HEX_RE.fullmatch(line):
            result.append(line)

    return result


def hex_bytes(line: str) -> list[int]:
    return [int(line[i:i + 2], 16) for i in range(0, len(line), 2)]


def find_response(raw: str, command: str) -> list[int] | None:
    cmd = command.replace(" ", "").upper()
    if len(cmd) < 4:
        return None

    mode = int(cmd[0:2], 16)
    pid = int(cmd[2:4], 16)
    expected_mode = mode + 0x40

    for line in clean_lines(raw, command):
        data = hex_bytes(line)

        # Search inside the line because some adapters include headers.
        for index in range(max(0, len(data) - 1)):
            if data[index] == expected_mode and data[index + 1] == pid:
                return data[index:]

    return None


def decode_pid(command: str, raw: str) -> str:
    response = find_response(raw, command)
    if not response or len(response) < 3:
        return "No data"

    pid = int(command[2:4], 16)
    data = response[2:]
    a = data[0] if len(data) > 0 else 0
    b = data[1] if len(data) > 1 else 0

    if pid == 0x02:
        if len(data) < 2:
            return "No data"
        return decode_dtc_bytes(a, b)

    if pid == 0x03:
        status = {
            0: "Open loop: insufficient temperature",
            1: "Closed loop",
            2: "Open loop: engine load/deceleration",
            4: "Open loop: system failure",
            8: "Closed loop with O2 fault",
        }
        values = []
        if a:
            values.append(status.get(a, f"Status {a}"))
        if b:
            values.append(status.get(b, f"Status {b}"))
        return ", ".join(values) if values else "No status"

    if pid == 0x04:
        return f"{a * 100 / 255:.1f}"

    if pid in (0x05, 0x0F):
        return str(a - 40)

    if pid == 0x06 or pid == 0x07:
        return f"{(a - 128) * 100 / 128:.1f}"

    if pid == 0x0B:
        return str(a)

    if pid == 0x0C:
        return f"{((a * 256) + b) / 4:.0f}"

    if pid == 0x0D:
        return str(a)

    if pid == 0x0E:
        return f"{a / 2 - 64:.1f}"

    if pid == 0x10:
        return f"{((a * 256) + b) / 100:.2f}"

    if pid == 0x11:
        return f"{a * 100 / 255:.1f}"

    if 0x14 <= pid <= 0x1B:
        voltage = a / 200
        trim = (b - 128) * 100 / 128
        return f"{voltage:.3f} V, trim {trim:.1f} %"

    return " ".join(f"{byte:02X}" for byte in data)


def decode_dtc_bytes(a: int, b: int) -> str:
    if a == 0 and b == 0:
        return ""

    prefixes = "PCBU"
    prefix = prefixes[(a >> 6) & 0x03]
    first_digit = (a >> 4) & 0x03
    remaining = ((a & 0x0F) << 8) | b
    return f"{prefix}{first_digit}{remaining:03X}"


def decode_dtcs(raw: str, command: str) -> list[str]:
    mode = int(command, 16)
    expected = mode + 0x40
    dtcs: list[str] = []

    for line in clean_lines(raw, command):
        data = hex_bytes(line)

        try:
            start = data.index(expected) + 1
        except ValueError:
            continue

        payload = data[start:]
        for i in range(0, len(payload) - 1, 2):
            code = decode_dtc_bytes(payload[i], payload[i + 1])
            if code and code not in dtcs:
                dtcs.append(code)

    return dtcs


class ELM327:
    def __init__(self) -> None:
        self.serial: serial.Serial | None = None
        self.lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    def connect(self, port: str, baudrate: int) -> list[str]:
        self.disconnect()
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.15,
            write_timeout=1,
        )
        time.sleep(0.25)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        commands = [
            "ATZ",    # reset
            "ATE0",   # echo off
            "ATL0",   # linefeeds off
            "ATS0",   # spaces off
            "ATH0",   # headers off
            "ATSP0",  # automatic protocol detection
            "0100",   # force ECU/protocol detection
            "ATDP",   # protocol description
        ]

        replies = []
        for command in commands:
            timeout = 8 if command == "0100" else 3
            replies.append(f"{command}: {self.send(command, timeout)}")

        return replies

    def disconnect(self) -> None:
        if self.serial:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        self.serial = None

    def send(self, command: str, timeout: float = 2.5) -> str:
        if not self.connected or not self.serial:
            raise RuntimeError("Adapter is not connected.")

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write((command.strip() + "\r").encode("ascii"))
            self.serial.flush()

            deadline = time.monotonic() + timeout
            received = bytearray()

            while time.monotonic() < deadline:
                chunk = self.serial.read(self.serial.in_waiting or 1)
                if chunk:
                    received.extend(chunk)
                    if b">" in received:
                        break
                else:
                    time.sleep(0.01)

            text = received.decode("ascii", errors="replace").replace(">", "").strip()
            return text


class ScannerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ELM327 Subaru Scanner")
        self.root.geometry("980x690")
        self.root.minsize(850, 580)

        self.elm = ELM327()
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.monitoring = False
        self.csv_file = None
        self.csv_writer = None
        self.log_path: Path | None = None

        self.port_var = tk.StringVar(value="COM4")
        self.baud_var = tk.StringVar(value="38400")
        self.status_var = tk.StringVar(value="Disconnected")
        self.interval_var = tk.StringVar(value="1.0")
        self.raw_var = tk.BooleanVar(value=False)

        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_messages)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Port").grid(row=0, column=0, padx=(0, 5))
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_combo.grid(row=0, column=1, padx=(0, 10))

        ttk.Label(top, text="Baud").grid(row=0, column=2, padx=(0, 5))
        ttk.Combobox(
            top,
            textvariable=self.baud_var,
            values=("38400", "9600", "115200"),
            width=10,
            state="readonly",
        ).grid(row=0, column=3, padx=(0, 10))

        ttk.Button(top, text="Refresh ports", command=self.refresh_ports).grid(row=0, column=4, padx=4)
        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=5, padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=6, padx=4)

        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=7, padx=(20, 0), sticky="w")
        top.columnconfigure(7, weight=1)

        actions = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        actions.pack(fill="x")

        ttk.Button(actions, text="Read DTCs", command=self.read_dtcs).pack(side="left", padx=4)
        ttk.Button(actions, text="Read Freeze Frame", command=self.read_freeze_frame).pack(side="left", padx=4)
        ttk.Button(actions, text="Clear DTCs", command=self.clear_dtcs).pack(side="left", padx=4)

        ttk.Separator(actions, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Label(actions, text="Interval").pack(side="left")
        ttk.Entry(actions, textvariable=self.interval_var, width=6).pack(side="left", padx=4)
        ttk.Label(actions, text="s").pack(side="left")
        ttk.Button(actions, text="Start monitoring", command=self.start_monitoring).pack(side="left", padx=4)
        ttk.Button(actions, text="Stop", command=self.stop_monitoring).pack(side="left", padx=4)
        ttk.Checkbutton(actions, text="Show raw responses", variable=self.raw_var).pack(side="left", padx=12)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        live_tab = ttk.Frame(notebook)
        dtc_tab = ttk.Frame(notebook)
        console_tab = ttk.Frame(notebook)

        notebook.add(live_tab, text="Live Data")
        notebook.add(dtc_tab, text="Codes / Freeze Frame")
        notebook.add(console_tab, text="Console")

        columns = ("parameter", "value", "unit")
        self.live_tree = ttk.Treeview(live_tab, columns=columns, show="headings")
        self.live_tree.heading("parameter", text="Parameter")
        self.live_tree.heading("value", text="Value")
        self.live_tree.heading("unit", text="Unit")
        self.live_tree.column("parameter", width=330)
        self.live_tree.column("value", width=250)
        self.live_tree.column("unit", width=100)
        self.live_tree.pack(fill="both", expand=True)

        for pid in LIVE_PIDS:
            self.live_tree.insert("", "end", iid=pid.command, values=(pid.name, "—", pid.unit))

        self.dtc_text = tk.Text(dtc_tab, wrap="word", font=("Consolas", 10))
        self.dtc_text.pack(fill="both", expand=True)

        self.console_text = tk.Text(console_tab, wrap="word", font=("Consolas", 10))
        self.console_text.pack(fill="both", expand=True)

        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bottom.pack(fill="x")
        ttk.Label(bottom, text="Log file:").pack(side="left")
        self.log_label = ttk.Label(bottom, text="not logging")
        self.log_label.pack(side="left", padx=5)
        ttk.Button(bottom, text="Choose CSV location", command=self.choose_log_path).pack(side="right")

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def run_background(self, target) -> None:
        threading.Thread(target=target, daemon=True).start()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Select a COM port.")
            return

        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Invalid baud", "Select a valid baud rate.")
            return

        self.status_var.set("Connecting…")

        def worker() -> None:
            try:
                replies = self.elm.connect(port, baud)
                protocol = replies[-1].split(":", 1)[-1].strip()
                self.messages.put(("connected", protocol))
                for reply in replies:
                    self.messages.put(("console", reply))
            except Exception as exc:
                self.elm.disconnect()
                self.messages.put(("error", f"Connection failed: {exc}"))

        self.run_background(worker)

    def disconnect(self) -> None:
        self.stop_monitoring()
        self.elm.disconnect()
        self.status_var.set("Disconnected")

    def ensure_connected(self) -> bool:
        if not self.elm.connected:
            messagebox.showerror("Not connected", "Connect to the ELM327 first.")
            return False
        return True

    def read_dtcs(self) -> None:
        if not self.ensure_connected():
            return

        def worker() -> None:
            try:
                stored_raw = self.elm.send("03", 4)
                pending_raw = self.elm.send("07", 4)
                permanent_raw = self.elm.send("0A", 4)

                stored = decode_dtcs(stored_raw, "03")
                pending = decode_dtcs(pending_raw, "07")
                permanent = decode_dtcs(permanent_raw, "0A")

                text = (
                    "STORED CODES\n"
                    + ("\n".join(stored) if stored else "None")
                    + "\n\nPENDING CODES\n"
                    + ("\n".join(pending) if pending else "None")
                    + "\n\nPERMANENT CODES\n"
                    + ("\n".join(permanent) if permanent else "None or unsupported")
                )
                self.messages.put(("dtc", text))
                if self.raw_var.get():
                    self.messages.put(("console", f"03 -> {stored_raw}\n07 -> {pending_raw}\n0A -> {permanent_raw}"))
            except Exception as exc:
                self.messages.put(("error", f"Could not read DTCs: {exc}"))

        self.run_background(worker)

    def read_freeze_frame(self) -> None:
        if not self.ensure_connected():
            return

        def worker() -> None:
            try:
                lines = ["FREEZE FRAME", ""]
                found = False

                for pid in FREEZE_PIDS:
                    raw = self.elm.send(pid.command, 3)
                    value = decode_pid(pid.command, raw)
                    if value != "No data":
                        found = True
                    lines.append(f"{pid.name}: {value} {pid.unit}".rstrip())

                    if self.raw_var.get():
                        self.messages.put(("console", f"{pid.command} -> {raw}"))

                if not found:
                    lines.append("\nNo freeze-frame data was returned by the ECU.")

                self.messages.put(("dtc", "\n".join(lines)))
            except Exception as exc:
                self.messages.put(("error", f"Could not read freeze frame: {exc}"))

        self.run_background(worker)

    def clear_dtcs(self) -> None:
        if not self.ensure_connected():
            return

        confirmed = messagebox.askyesno(
            "Clear DTCs",
            "This clears stored codes and freeze-frame data.\n\nContinue?",
        )
        if not confirmed:
            return

        def worker() -> None:
            try:
                raw = self.elm.send("04", 5)
                self.messages.put(("dtc", f"CLEAR DTC RESPONSE\n\n{raw or 'No response'}"))
                self.messages.put(("console", f"04 -> {raw}"))
            except Exception as exc:
                self.messages.put(("error", f"Could not clear DTCs: {exc}"))

        self.run_background(worker)

    def choose_log_path(self) -> None:
        suggested = f"subaru_obd_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        filename = filedialog.asksaveasfilename(
            title="Choose CSV log file",
            defaultextension=".csv",
            initialfile=suggested,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if filename:
            self.log_path = Path(filename)
            self.log_label.config(text=str(self.log_path))

    def open_csv(self) -> None:
        if self.log_path is None:
            self.log_path = Path.cwd() / f"subaru_obd_log_{datetime.now():%Y%m%d_%H%M%S}.csv"

        self.csv_file = self.log_path.open("w", newline="", encoding="utf-8-sig")
        fieldnames = ["timestamp"] + [pid.name for pid in LIVE_PIDS]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames, delimiter=";")
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.messages.put(("log_path", str(self.log_path)))

    def close_csv(self) -> None:
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None

    def start_monitoring(self) -> None:
        if not self.ensure_connected() or self.monitoring:
            return

        try:
            interval = max(0.2, float(self.interval_var.get().replace(",", ".")))
        except ValueError:
            messagebox.showerror("Invalid interval", "Enter a number such as 1.0.")
            return

        self.stop_event.clear()
        self.monitoring = True
        self.open_csv()
        self.status_var.set("Monitoring")

        def worker() -> None:
            try:
                while not self.stop_event.is_set():
                    cycle_start = time.monotonic()
                    row = {"timestamp": datetime.now().isoformat(timespec="milliseconds")}

                    for pid in LIVE_PIDS:
                        if self.stop_event.is_set():
                            break

                        raw = self.elm.send(pid.command, 2.5)
                        value = decode_pid(pid.command, raw)
                        row[pid.name] = value
                        self.messages.put(("live", (pid.command, value)))

                        if self.raw_var.get():
                            self.messages.put(("console", f"{pid.command} -> {raw}"))

                    if self.csv_writer:
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()

                    remaining = interval - (time.monotonic() - cycle_start)
                    if remaining > 0:
                        self.stop_event.wait(remaining)
            except Exception as exc:
                self.messages.put(("error", f"Monitoring stopped: {exc}"))
            finally:
                self.monitoring = False
                self.close_csv()
                self.messages.put(("monitor_stopped", None))

        self.run_background(worker)

    def stop_monitoring(self) -> None:
        self.stop_event.set()

    def process_messages(self) -> None:
        try:
            while True:
                kind, payload = self.messages.get_nowait()

                if kind == "connected":
                    self.status_var.set(f"Connected: {payload}")

                elif kind == "console":
                    self.console_text.insert("end", str(payload) + "\n")
                    self.console_text.see("end")

                elif kind == "dtc":
                    self.dtc_text.delete("1.0", "end")
                    self.dtc_text.insert("1.0", str(payload))

                elif kind == "live":
                    command, value = payload
                    if self.live_tree.exists(command):
                        current = self.live_tree.item(command, "values")
                        self.live_tree.item(command, values=(current[0], value, current[2]))

                elif kind == "log_path":
                    self.log_label.config(text=str(payload))

                elif kind == "monitor_stopped":
                    if self.elm.connected:
                        self.status_var.set("Connected")
                    self.monitoring = False

                elif kind == "error":
                    self.status_var.set("Error")
                    messagebox.showerror("Error", str(payload))

        except queue.Empty:
            pass

        self.root.after(100, self.process_messages)

    def close(self) -> None:
        self.stop_event.set()
        self.elm.disconnect()
        self.close_csv()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ScannerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
