from __future__ import annotations

import csv
import math
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from typing import Deque, Dict, List, Optional

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from serial.tools import list_ports

DEFAULT_BAUDRATE = 115200
DEFAULT_WINDOW = 500
DEFAULT_INTERVAL_MS = 100
DEFAULT_ENCODING = "utf-8"
MIN_REFERENCE = 0.0
MAX_REFERENCE = 30.0
GENERATOR_INTERVAL_MS = 25
MAX_LINES_PER_POLL = 100

CHANNELS = ["Posicion", "Referencia", "usat"]
CHANNEL_ALIASES = {
    "posicion": "Posicion",
    "referencia": "Referencia",
    "usat": "usat",
}


@dataclass
class Sample:
    timestamp: float
    values: Dict[str, float]


class SerialPlotterApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Serial Plotter")

        self.serial_conn: Optional[serial.Serial] = None
        self.encoding = DEFAULT_ENCODING
        self.interval_ms = DEFAULT_INTERVAL_MS

        self.sample_window: Deque[Sample] = deque(maxlen=DEFAULT_WINDOW)
        self.history: List[Sample] = []

        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.window_var = tk.IntVar(value=DEFAULT_WINDOW)
        self.status_var = tk.StringVar(value="Disconnected")
        self.command_var = tk.StringVar()
        self.signal_type_var = tk.StringVar(value="Sine")
        self.frequency_var = tk.DoubleVar(value=1.0)
        self.max_value_var = tk.DoubleVar(value=10.0)

        self.generator_running = False
        self.generator_job: Optional[str] = None
        self.generator_start_time = 0.0

        self._build_ui()
        self.refresh_ports()
        self.schedule_poll()

    def _build_ui(self) -> None:
        control_frame = ttk.LabelFrame(self.root, text="Connection")
        control_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(control_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="we")

        refresh_btn = ttk.Button(control_frame, text="Refresh", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(control_frame, text="Baud:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        baud_entry = ttk.Entry(control_frame, textvariable=self.baud_var, width=10)
        baud_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(control_frame, text="Window:").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        window_spin = ttk.Spinbox(
            control_frame,
            from_=10,
            to=10000,
            textvariable=self.window_var,
            width=7,
            command=self._sync_window_size,
        )
        window_spin.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        connect_btn = ttk.Button(control_frame, text="Connect", command=self.connect)
        connect_btn.grid(row=0, column=3, padx=5, pady=5)

        disconnect_btn = ttk.Button(control_frame, text="Disconnect", command=self.disconnect)
        disconnect_btn.grid(row=0, column=4, padx=5, pady=5)

        export_btn = ttk.Button(control_frame, text="Export CSV", command=self.export_csv)
        export_btn.grid(row=1, column=4, padx=5, pady=5)

        control_frame.columnconfigure(1, weight=1)

        command_frame = ttk.LabelFrame(self.root, text="Command")
        command_frame.pack(fill="x", padx=10, pady=5)
        command_entry = ttk.Entry(command_frame, textvariable=self.command_var)
        command_entry.grid(row=0, column=0, padx=5, pady=5, sticky="we")
        command_entry.bind("<Return>", self.send_command)
        send_btn = ttk.Button(command_frame, text="Send", command=self.send_command)
        send_btn.grid(row=0, column=1, padx=5, pady=5)
        command_frame.columnconfigure(0, weight=1)

        generator_frame = ttk.LabelFrame(self.root, text="Signal Generator (r XX.XX)")
        generator_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(generator_frame, text="Waveform:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        waveform_combo = ttk.Combobox(
            generator_frame,
            textvariable=self.signal_type_var,
            state="readonly",
            values=["Sine", "Square"],
            width=10,
        )
        waveform_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(generator_frame, text="Frequency (Hz):").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        freq_entry = ttk.Entry(generator_frame, textvariable=self.frequency_var, width=8)
        freq_entry.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        ttk.Label(generator_frame, text="Max value:").grid(row=0, column=4, padx=5, pady=5, sticky="e")
        max_spin = ttk.Spinbox(
            generator_frame,
            from_=MIN_REFERENCE,
            to=MAX_REFERENCE,
            increment=0.5,
            textvariable=self.max_value_var,
            width=6,
        )
        max_spin.grid(row=0, column=5, padx=5, pady=5, sticky="w")

        start_btn = ttk.Button(generator_frame, text="Start", command=self.start_generator)
        start_btn.grid(row=0, column=6, padx=5, pady=5)
        stop_btn = ttk.Button(generator_frame, text="Stop", command=self.stop_generator)
        stop_btn.grid(row=0, column=7, padx=5, pady=5)

        plot_frame = ttk.LabelFrame(self.root, text="Live Plot")
        plot_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.figure = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.lines = {}
        for channel in CHANNELS:
            (line,) = self.ax.plot([], [], lw=1.5, label=channel)
            self.lines[channel] = line
        self.ax.set_xlabel("Sample # (recent on the right)")
        self.ax.set_ylabel("Value")
        self.ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)

        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Label(status_frame, textvariable=self.status_var).pack(anchor="w")

    def refresh_ports(self) -> None:
        ports = [self._preferred_device(port.device) for port in list_ports.comports()]
        self.port_combo["values"] = ports or ["<no ports>"]
        if ports:
            self.port_combo.current(0)
        else:
            self.port_var.set("")
        self.set_status(f"Found {len(ports)} port(s)")

    def connect(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.set_status("Already connected")
            return

        port = self.port_var.get().strip()
        if not port or port == "<no ports>":
            messagebox.showwarning("Serial Plotter", "Please select a valid serial port.")
            return

        try:
            baud = int(self.baud_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Serial Plotter", "Baud rate must be an integer.")
            return

        self._sync_window_size()

        try:
            self.serial_conn = serial.Serial(port, baudrate=baud, timeout=0.1)
        except serial.SerialException as exc:
            messagebox.showerror("Serial Plotter", f"Could not open port {port}: {exc}")
            self.serial_conn = None
            return

        self.sample_window.clear()
        self.history.clear()
        self.set_status(f"Connected to {port} @ {baud} baud")

    def disconnect(self) -> None:
        self.stop_generator()
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except serial.SerialException:
                pass
            finally:
                self.serial_conn = None
        self.set_status("Disconnected")

    def send_command(self, *_args) -> None:
        command = self.command_var.get().strip()
        if not command:
            return
        if self._send_serial_command(command):
            self.command_var.set("")

    def start_generator(self) -> None:
        if self.generator_running:
            return
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Serial Plotter", "Connect before starting the generator.")
            return
        try:
            frequency = float(self.frequency_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Serial Plotter", "Frequency must be numeric.")
            return
        if frequency <= 0:
            messagebox.showerror("Serial Plotter", "Frequency must be greater than zero.")
            return
        try:
            max_value = float(self.max_value_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Serial Plotter", "Max value must be numeric.")
            return
        clamped_max = max(MIN_REFERENCE, min(MAX_REFERENCE, max_value))
        self.max_value_var.set(clamped_max)
        self.generator_running = True
        self.generator_start_time = time.perf_counter()
        self.set_status(
            f"Generator running ({self.signal_type_var.get()} @ {frequency:.2f} Hz, max {clamped_max:.2f})"
        )
        self._generator_tick()

    def stop_generator(self) -> None:
        if self.generator_job is not None:
            self.root.after_cancel(self.generator_job)
            self.generator_job = None
        if self.generator_running:
            self.generator_running = False
            self.set_status("Signal generator stopped")

    def _schedule_generator_tick(self) -> None:
        if self.generator_running:
            self.generator_job = self.root.after(GENERATOR_INTERVAL_MS, self._generator_tick)

    def _generator_tick(self) -> None:
        if not self.generator_running:
            return
        if not self.serial_conn or not self.serial_conn.is_open:
            self.stop_generator()
            return
        try:
            frequency = float(self.frequency_var.get())
            max_value = float(self.max_value_var.get())
        except (tk.TclError, ValueError):
            self.stop_generator()
            return
        if frequency <= 0:
            self.stop_generator()
            return
        clamped_max = max(MIN_REFERENCE, min(MAX_REFERENCE, max_value))
        self.max_value_var.set(clamped_max)
        elapsed = time.perf_counter() - self.generator_start_time
        waveform = self.signal_type_var.get().strip().lower()
        value = self._compute_waveform_value(waveform, frequency, elapsed, clamped_max)
        self.send_reference(value)
        self._schedule_generator_tick()

    def _compute_waveform_value(self, waveform: str, frequency: float, elapsed: float, max_value: float) -> float:
        if waveform == "square":
            return max_value if math.sin(2 * math.pi * frequency * elapsed) >= 0 else MIN_REFERENCE
        return (max_value / 2.0) * (math.sin(2 * math.pi * frequency * elapsed) + 1)

    def send_reference(self, value: float) -> None:
        clamped = max(MIN_REFERENCE, min(MAX_REFERENCE, value))
        command = f"r {clamped:.2f}"
        self._send_serial_command(command, quiet=True)

    def export_csv(self) -> None:
        if not self.history:
            messagebox.showinfo("Serial Plotter", "No data available to export.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Save data as CSV",
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                header = ["index", "timestamp", *CHANNELS]
                writer.writerow(header)
                for idx, sample in enumerate(self.history):
                    row = [idx, sample.timestamp]
                    row.extend(sample.values.get(channel, "") for channel in CHANNELS)
                    writer.writerow(row)
        except OSError as exc:
            messagebox.showerror("Serial Plotter", f"Error writing file: {exc}")
            return

        self.set_status(f"Saved {len(self.history)} samples to {file_path}")

    def schedule_poll(self) -> None:
        self.root.after(self.interval_ms, self.poll_serial)

    def poll_serial(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.read_samples()
            except serial.SerialException as exc:
                self.set_status(f"Serial error: {exc}")
                self.disconnect()
        self.schedule_poll()

    def read_samples(self) -> None:
        assert self.serial_conn is not None
        lines_read = 0
        while lines_read < MAX_LINES_PER_POLL:
            raw = self.serial_conn.readline()
            if not raw:
                break
            lines_read += 1
            try:
                text = raw.decode(self.encoding, errors="ignore").strip()
                if not text:
                    continue
                parsed = self.parse_values(text)
                if not parsed or not all(channel in parsed for channel in CHANNELS):
                    continue
            except ValueError:
                continue
            self._append_sample(parsed)

    def _append_sample(self, values: Dict[str, float]) -> None:
        timestamp = time.time()
        sample_values = {}
        for channel in CHANNELS:
            value = values.get(channel)
            if value is None:
                continue
            sample_values[channel] = value
        sample = Sample(timestamp=timestamp, values=sample_values)
        self.sample_window.append(sample)
        self.history.append(sample)
        self.update_plot()

    def update_plot(self) -> None:
        if not self.sample_window:
            return

        x_data = list(range(len(self.sample_window)))
        for channel, line in self.lines.items():
            y_data = [sample.values.get(channel, float("nan")) for sample in self.sample_window]
            line.set_data(x_data, y_data)

        window_cap = self.sample_window.maxlen or DEFAULT_WINDOW
        upper_x = max(len(self.sample_window) - 1, window_cap - 1)
        self.ax.set_xlim(0, max(upper_x, 1))
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def _sync_window_size(self) -> None:
        try:
            window_size = max(1, int(self.window_var.get()))
        except (tk.TclError, ValueError):
            window_size = DEFAULT_WINDOW
            self.window_var.set(window_size)
        if window_size != (self.sample_window.maxlen or DEFAULT_WINDOW):
            self.sample_window = deque(self.sample_window, maxlen=window_size)
        self.set_status(f"Window size set to {window_size} samples")

    def set_status(self, message: str) -> None:
        self.status_var.set(message)

    def _send_serial_command(self, command: str, quiet: bool = False) -> bool:
        if not self.serial_conn or not self.serial_conn.is_open:
            if not quiet:
                messagebox.showwarning("Serial Plotter", "Connect to a serial device before sending commands.")
            return False
        payload = f"{command}\n".encode(self.encoding)
        try:
            self.serial_conn.write(payload)
            self.serial_conn.flush()
        except serial.SerialException as exc:
            messagebox.showerror("Serial Plotter", f"Failed to send command: {exc}")
            self.disconnect()
            return False
        if not quiet:
            self.set_status(f"Sent command: {command}")
        return True

    @staticmethod
    def _preferred_device(device: str) -> str:
        if device.startswith("/dev/cu."):
            maybe_tty = device.replace("/dev/cu.", "/dev/tty.", 1)
            return maybe_tty
        return device

    @staticmethod
    def parse_values(text: str) -> Optional[Dict[str, float]]:
        if ":" not in text:
            return None
        values: Dict[str, float] = {}
        for pair in text.split(","):
            part = pair.strip()
            if ":" not in part:
                continue
            key, raw_val = part.split(":", 1)
            normalized_key = SerialPlotterApp._normalize_key(key)
            canonical = CHANNEL_ALIASES.get(normalized_key)
            if not canonical:
                continue
            try:
                values[canonical] = float(raw_val.strip())
            except ValueError:
                return None
        return values if values else None

    @staticmethod
    def _normalize_key(key: str) -> str:
        stripped = key.strip().lstrip(" >#:@")
        return stripped.lower()


def main() -> None:
    root = tk.Tk()
    SerialPlotterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
