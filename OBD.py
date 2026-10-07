#!/usr/bin/env python3
"""
ELM327 OBD-II Scanner / Logger v3

Supported setups:
- Windows, USB ELM327 (FTDI / PIC18F25K80), typical port COM4, 38400 baud
- Ford and other CAN vehicles (ISO 15765-4, ~2008 onwards)
- Subaru and other K-line vehicles (ISO 9141-2 / KWP2000)

Dependency:
    py -m pip install pyserial
"""

from __future__ import annotations

import csv
import queue
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from serial.tools import list_ports

from obd_core import (
    DTC, ELM327, PID_CATALOG, QUICK_PID_IDS, SUPPORT_PIDS, VEHICLE_PROFILES,
    clear_dtcs, decode_pid_value, derive_values, discover_supported_pids, ecu_name,
    format_readiness, pid_data, read_freeze_frame, read_generic_dtcs,
    read_monitor_status, read_vehicle_info,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ELM327 OBD-II Scanner / Logger v3")
        self.root.geometry("1180x780")
        self.root.minsize(940, 640)

        self.elm = ELM327()
        self.elm.on_raw = self.on_raw
        self.queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.busy = threading.Lock()
        self.monitoring = False
        self.supported: set[int] = set()
        self.ecm_address: str | None = None
        self.vehicle: dict[str, str] = {}
        self.dtcs: list[DTC] = []
        self.dtc_sections: dict[str, str] = {}
        self.log_path: Path | None = None
        self.csv_handle = None
        self.csv_writer = None

        self.port_var = tk.StringVar(value="COM4")
        self.baud_var = tk.StringVar(value="38400")
        self.vehicle_var = tk.StringVar(value="Auto detect")
        self.profile_var = tk.StringVar(value="Quick diagnostic")
        self.interval_var = tk.StringVar(value="0.0")
        self.raw_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Disconnected")
        self.vehicle_info_var = tk.StringVar(value="No vehicle connected")
        self.summary_var = tk.StringVar(value="No ECU data")

        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ------------------------------------------------------------------ UI

    def build_ui(self) -> None:
        connection = ttk.Frame(self.root, padding=(10, 10, 10, 4))
        connection.pack(fill="x")

        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(0, 4))
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=10)
        self.port_box.grid(row=0, column=1, padx=(0, 8))

        ttk.Label(connection, text="Baud").grid(row=0, column=2, padx=(0, 4))
        ttk.Combobox(connection, textvariable=self.baud_var,
                     values=("38400", "9600", "115200", "230400", "500000"),
                     state="readonly", width=8).grid(row=0, column=3, padx=(0, 8))

        ttk.Label(connection, text="Vehicle").grid(row=0, column=4, padx=(0, 4))
        ttk.Combobox(connection, textvariable=self.vehicle_var, values=list(VEHICLE_PROFILES),
                     state="readonly", width=16).grid(row=0, column=5, padx=(0, 8))

        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=6, padx=3)
        ttk.Button(connection, text="Connect", command=self.connect).grid(row=0, column=7, padx=3)
        ttk.Button(connection, text="Disconnect", command=self.disconnect).grid(row=0, column=8, padx=3)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=9, padx=(15, 0), sticky="w")
        connection.columnconfigure(9, weight=1)

        info = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        info.pack(fill="x")
        ttk.Label(info, textvariable=self.vehicle_info_var, foreground="#315A7D").pack(side="left")
        ttk.Checkbutton(info, text="Show raw responses in console", variable=self.raw_var).pack(side="right")

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        self.notebook = notebook

        dtc_tab = ttk.Frame(notebook, padding=6)
        live_tab = ttk.Frame(notebook, padding=6)
        diagnosis_tab = ttk.Frame(notebook)
        console_tab = ttk.Frame(notebook)
        notebook.add(dtc_tab, text="Trouble codes")
        notebook.add(live_tab, text="Live data")
        notebook.add(diagnosis_tab, text="Derived / diagnosis")
        notebook.add(console_tab, text="Console")

        self.build_dtc_tab(dtc_tab)
        self.build_live_tab(live_tab)

        self.diagnosis_text = tk.Text(diagnosis_tab, wrap="word", font=("Consolas", 10))
        self.diagnosis_text.pack(fill="both", expand=True)

        self.console_text = tk.Text(console_tab, wrap="word", font=("Consolas", 9))
        self.console_text.pack(fill="both", expand=True)

        bottom = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        bottom.pack(fill="x")
        ttk.Label(bottom, textvariable=self.summary_var).pack(side="left")

    def build_dtc_tab(self, tab: ttk.Frame) -> None:
        toolbar = ttk.Frame(tab)
        toolbar.pack(fill="x", pady=(0, 6))
        ttk.Button(toolbar, text="Read codes", command=self.read_dtcs).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Read freeze frame", command=self.read_freeze).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Clear codes", command=self.clear_dtcs).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Save report", command=self.save_report).pack(side="right", padx=3)

        panes = ttk.Panedwindow(tab, orient="vertical")
        panes.pack(fill="both", expand=True)

        table = ttk.Frame(panes)
        self.dtc_tree = ttk.Treeview(table, columns=("code", "status", "module", "description"), show="headings", height=8)
        for key, title, width, stretch in (
            ("code", "Code", 90, False), ("status", "Status", 120, False),
            ("module", "Module", 190, False), ("description", "Description", 600, True),
        ):
            self.dtc_tree.heading(key, text=title)
            self.dtc_tree.column(key, width=width, stretch=stretch)
        self.dtc_tree.tag_configure("Stored", foreground="#A44949")
        self.dtc_tree.tag_configure("Active", foreground="#A44949")
        self.dtc_tree.tag_configure("Permanent", foreground="#7A3B3B")
        self.dtc_tree.tag_configure("Pending", foreground="#8A5D20")
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.dtc_tree.yview)
        self.dtc_tree.configure(yscrollcommand=scroll.set)
        self.dtc_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        panes.add(table, weight=1)

        self.dtc_text = tk.Text(tab, wrap="word", font=("Consolas", 10), height=12)
        panes.add(self.dtc_text, weight=1)

    def build_live_tab(self, tab: ttk.Frame) -> None:
        toolbar = ttk.Frame(tab)
        toolbar.pack(fill="x", pady=(0, 6))

        ttk.Label(toolbar, text="Profile").pack(side="left")
        ttk.Combobox(toolbar, textvariable=self.profile_var,
                     values=("Quick diagnostic", "All supported PIDs"),
                     state="readonly", width=19).pack(side="left", padx=4)
        ttk.Label(toolbar, text="Pause between requests").pack(side="left", padx=(8, 0))
        ttk.Entry(toolbar, textvariable=self.interval_var, width=6).pack(side="left", padx=4)
        ttk.Label(toolbar, text="s").pack(side="left")
        ttk.Button(toolbar, text="Start logging", command=self.start_monitoring).pack(side="left", padx=(12, 4))
        ttk.Button(toolbar, text="Stop", command=self.stop_monitoring).pack(side="left", padx=4)

        self.log_label = ttk.Label(toolbar, text="Log: automatic")
        ttk.Button(toolbar, text="Choose CSV", command=self.choose_log).pack(side="right")
        self.log_label.pack(side="right", padx=6)

        frame = ttk.Frame(tab)
        frame.pack(fill="both", expand=True)
        self.live_tree = ttk.Treeview(frame, columns=("pid", "name", "value", "unit", "support"), show="headings")
        for key, title, width in (
            ("pid", "PID", 70), ("name", "Parameter", 430),
            ("value", "Value", 280), ("unit", "Unit", 90), ("support", "Supported", 90),
        ):
            self.live_tree.heading(key, text=title)
            self.live_tree.column(key, width=width)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.live_tree.yview)
        self.live_tree.configure(yscrollcommand=scroll.set)
        self.live_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    # ------------------------------------------------------------- helpers

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def on_raw(self, command: str, response: str) -> None:
        if self.raw_var.get():
            self.queue.put(("console", f"{command} -> {response}"))

    @property
    def make(self) -> str:
        return self.vehicle.get("make", "")

    def background(self, func, name: str) -> None:
        """Run adapter work in a thread; refuse to overlap with other adapter jobs."""
        if not self.elm.connected and name != "connect":
            messagebox.showerror("Not connected", "Connect to the ELM327 first.")
            return
        if self.monitoring and name != "connect":
            messagebox.showinfo("Busy", "Stop live logging first.")
            return
        if not self.busy.acquire(blocking=False):
            messagebox.showinfo("Busy", "The adapter is still busy with the previous request.")
            return

        def run() -> None:
            try:
                func()
            except Exception as exc:
                self.queue.put(("error", f"{name.capitalize()} failed:\n{exc}"))
            finally:
                self.busy.release()
                self.queue.put(("idle", None))

        self.status_var.set(f"{name.capitalize()}…")
        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------ actions

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
        protocol = VEHICLE_PROFILES.get(self.vehicle_var.get(), "0")

        def worker() -> None:
            try:
                for line in self.elm.connect(port, baud, protocol):
                    self.queue.put(("console", line))
                primary, supported = discover_supported_pids(self.elm)
                vehicle = read_vehicle_info(self.elm)
            except Exception:
                self.elm.disconnect()
                raise
            self.queue.put(("connected", (primary, supported, vehicle)))

        self.background(worker, "connect")

    def disconnect(self) -> None:
        self.stop_monitoring()
        self.elm.disconnect()
        self.supported.clear()
        self.vehicle = {}
        self.status_var.set("Disconnected")
        self.vehicle_info_var.set("No vehicle connected")
        self.summary_var.set("No ECU data")

    def read_dtcs(self) -> None:
        def worker() -> None:
            status = read_monitor_status(self.elm)
            dtcs, notes = read_generic_dtcs(self.elm, self.make)
            details = [format_readiness(status)]
            if notes:
                details += ["", "Notes:"] + [f"  {note}" for note in notes]
            self.queue.put(("dtcs", (dtcs, "\n".join(details))))

        self.background(worker, "read codes")

    def read_freeze(self) -> None:
        pids = sorted(self.supported)

        def worker() -> None:
            lines = read_freeze_frame(self.elm, pids, self.ecm_address, self.make)
            self.queue.put(("dtc_section", ("freeze", "\n".join(lines))))

        self.background(worker, "freeze frame")

    def clear_dtcs(self) -> None:
        if not self.elm.connected:
            messagebox.showerror("Not connected", "Connect to the ELM327 first.")
            return
        if not messagebox.askyesno(
            "Clear DTCs",
            "This clears stored codes, readiness monitors and freeze-frame data.\n"
            "Save a report first if you still need them.\n\n"
            "Ignition ON, engine OFF. Continue?",
        ):
            return

        def worker() -> None:
            result = clear_dtcs(self.elm)
            self.queue.put(("dtcs", ([], f"CLEAR DTC RESPONSE\n\n{result}")))

        self.background(worker, "clear codes")

    def save_report(self) -> None:
        if not self.dtcs and not self.dtc_sections:
            messagebox.showinfo("Nothing to save", "Read codes first.")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="Save trouble code report", defaultextension=".txt", initialdir=LOG_DIR,
            initialfile=f"dtc_report_{datetime.now():%Y%m%d_%H%M%S}.txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
        )
        if not filename:
            return
        lines = [f"Trouble code report – {datetime.now():%Y-%m-%d %H:%M:%S}",
                 self.vehicle_info_var.get(), ""]
        if self.dtcs:
            for dtc in self.dtcs:
                detail = f" [{dtc.detail}]" if dtc.detail else ""
                lines.append(f"{dtc.code:<9} {dtc.status:<10} {dtc.module:<28} {dtc.description}{detail}")
        else:
            lines.append("No trouble codes reported.")
        lines += ["", self.dtc_details]
        Path(filename).write_text("\n".join(lines), encoding="utf-8")
        messagebox.showinfo("Report saved", f"Saved to:\n{filename}")

    def populate_tree(self) -> None:
        self.live_tree.delete(*self.live_tree.get_children())
        for pid in sorted(PID_CATALOG):
            definition = PID_CATALOG[pid]
            supported = "Yes" if pid in self.supported else "No"
            self.live_tree.insert(
                "", "end", iid=f"{pid:02X}",
                values=(f"01 {pid:02X}", definition.name, "—", definition.unit, supported),
            )

    def selected_pids(self) -> list[int]:
        usable = [pid for pid in sorted(self.supported) if pid in PID_CATALOG and pid not in SUPPORT_PIDS]
        if self.profile_var.get() == "All supported PIDs":
            return usable
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
            self.log_label.config(text=f"Log: {self.log_path.name}")

    def start_monitoring(self) -> None:
        if not self.elm.connected:
            messagebox.showerror("Not connected", "Connect to the ELM327 first.")
            return
        if self.monitoring:
            return
        if not self.busy.acquire(blocking=False):
            messagebox.showinfo("Busy", "The adapter is still busy with the previous request.")
            return

        pids = self.selected_pids()
        if not pids:
            self.busy.release()
            messagebox.showerror("No supported PIDs", "No PIDs are available for this profile.")
            return

        try:
            pause = max(0.0, float(self.interval_var.get().replace(",", ".")))
        except ValueError:
            self.busy.release()
            messagebox.showerror("Invalid pause", "Use a number such as 0.3.")
            return

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = self.log_path or LOG_DIR / f"obd_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.log_path = None  # a chosen file is used once; the next run gets a fresh name

        self.csv_handle = log_path.open("w", newline="", encoding="utf-8-sig")
        fields = ["timestamp"] + [PID_CATALOG[p].name for p in pids] + [
            "Derived boost relative to atmosphere (kPa)",
            "Derived boost relative to atmosphere (bar)",
            "Combined fuel trim Bank 1 (%)",
        ]
        self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=fields, delimiter=";")
        self.csv_writer.writeheader()
        self.log_label.config(text=f"Log: {log_path.name}")

        self.stop_event.clear()
        self.monitoring = True
        self.status_var.set(f"Logging {len(pids)} PIDs")
        address = self.ecm_address

        def worker() -> None:
            try:
                while not self.stop_event.is_set():
                    row: dict[str, object] = {"timestamp": datetime.now().isoformat(timespec="milliseconds")}
                    cycle_values: dict[int, tuple[str, float | None]] = {}

                    for pid in pids:
                        if self.stop_event.is_set():
                            break
                        data = pid_data(self.elm.request(f"01{pid:02X}", 3.0), 0x01, pid, address)
                        text, numeric = decode_pid_value(pid, data) if data is not None else ("No data", None)
                        cycle_values[pid] = (text, numeric)
                        row[PID_CATALOG[pid].name] = text
                        self.queue.put(("live", (pid, text)))
                        if pause:
                            self.stop_event.wait(pause)

                    derived = derive_values(cycle_values, self.make)
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
                self.busy.release()
                self.queue.put(("idle", None))

        threading.Thread(target=worker, daemon=True).start()

    def stop_monitoring(self) -> None:
        self.stop_event.set()

    # ---------------------------------------------------------- UI updates

    @property
    def dtc_details(self) -> str:
        return "\n\n".join(text for text in self.dtc_sections.values() if text)

    def set_dtc_section(self, key: str, text: str) -> None:
        self.dtc_sections[key] = text
        self.set_text(self.dtc_text, self.dtc_details)

    def show_dtcs(self, dtcs: list[DTC], details: str) -> None:
        self.dtcs = dtcs
        self.dtc_sections = {}
        self.dtc_tree.delete(*self.dtc_tree.get_children())
        for dtc in dtcs:
            description = dtc.description + (f"  [{dtc.detail}]" if dtc.detail else "")
            self.dtc_tree.insert("", "end", values=(dtc.code, dtc.status, dtc.module, description),
                                 tags=(dtc.status,))
        if not dtcs:
            self.dtc_tree.insert("", "end", values=("—", "", "", "No trouble codes reported."))
        self.set_dtc_section("codes", details)

    @staticmethod
    def set_text(widget: tk.Text, text: str) -> None:
        widget.delete("1.0", "end")
        widget.insert("1.0", text)

    def process_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()

                if kind == "connected":
                    primary, supported, vehicle = payload
                    self.ecm_address = primary or None
                    self.supported = set(supported.get(primary, set()))
                    self.vehicle = vehicle
                    self.populate_tree()
                    known = sum(1 for pid in self.supported if pid in PID_CATALOG)
                    parts = [vehicle.get("make") or "Unknown make"]
                    if vehicle.get("vin"):
                        parts.append(f"VIN {vehicle['vin']}")
                    if vehicle.get("ecu_name"):
                        parts.append(vehicle["ecu_name"])
                    parts += [self.elm.protocol_name, self.elm.version]
                    self.vehicle_info_var.set(" · ".join(parts))
                    modules = ", ".join(ecu_name(address) for address in supported)
                    self.summary_var.set(f"{len(supported)} module(s) answered ({modules}); "
                                         f"engine reports {len(self.supported)} PIDs, {known} decoded")
                elif kind == "live":
                    pid, value = payload
                    iid = f"{pid:02X}"
                    if self.live_tree.exists(iid):
                        old = self.live_tree.item(iid, "values")
                        self.live_tree.item(iid, values=(old[0], old[1], value, old[3], old[4]))
                elif kind == "diagnosis":
                    self.set_text(self.diagnosis_text, str(payload))
                elif kind == "dtcs":
                    self.show_dtcs(*payload)
                elif kind == "dtc_section":
                    self.set_dtc_section(*payload)
                elif kind == "console":
                    self.console_text.insert("end", str(payload) + "\n")
                    self.console_text.see("end")
                elif kind == "idle":
                    self.status_var.set(f"Connected: {self.elm.protocol_name}" if self.elm.connected
                                        else "Disconnected")
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


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
