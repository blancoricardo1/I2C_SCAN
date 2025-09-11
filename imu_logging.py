import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import csv

class SerialLoggerApp:
    def __init__(self, master):
        self.master = master
        self.master.title("IMU Serial Logger")
        self.ser = None
        self.stop_thread = False

        # UI
        self.port_label = ttk.Label(master, text="Select COM Port:")
        self.port_label.pack(pady=(10, 0))

        self.port_combo = ttk.Combobox(master, values=self.list_ports(), state="readonly", width=30)
        self.port_combo.pack(pady=5)

        self.connect_button = ttk.Button(master, text="Connect & Start Logging", command=self.connect)
        self.connect_button.pack(pady=10)

        self.log_display = tk.Text(master, height=15, width=80, state="disabled")
        self.log_display.pack(padx=10, pady=10)

        self.stop_button = ttk.Button(master, text="Stop Logging", command=self.stop_logging, state="disabled")
        self.stop_button.pack(pady=(0, 10))

    def list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self):
        selected_port = self.port_combo.get()
        if not selected_port:
            messagebox.showerror("Error", "Please select a COM port.")
            return

        try:
            self.ser = serial.Serial(selected_port, 115200, timeout=1)
            self.stop_thread = False
            self.stop_button.config(state="normal")
            self.connect_button.config(state="disabled")
            self.log_display.config(state="normal")
            self.log_display.insert(tk.END, f"Connected to {selected_port} at 115200 baud.\n")
            self.log_display.insert(tk.END, "Logging started...\n")
            self.log_display.config(state="disabled")

            filename = f"imu_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.log_thread = threading.Thread(target=self.log_serial, args=(filename,), daemon=True)
            self.log_thread.start()
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", str(e))

    def parse_imu_line(self, line):
        parts = line.split(',')
        if len(parts) != 6:
            return ['', '', '', '', '', '']
        try:
            return [float(x.strip()) for x in parts]
        except ValueError:
            return ['', '', '', '', '', '']

    def log_serial(self, filename):
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])

            while not self.stop_thread:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        values = self.parse_imu_line(line)
                        self.log_display.config(state="normal")
                        self.log_display.insert(tk.END, f"{values}\n")
                        self.log_display.see(tk.END)
                        self.log_display.config(state="disabled")
                        writer.writerow(values)
                        f.flush()
                except Exception as e:
                    print(f"Error during logging: {e}")
                    break

    def stop_logging(self):
        self.stop_thread = True
        self.connect_button.config(state="normal")
        self.stop_button.config(state="disabled")
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.log_display.config(state="normal")
        self.log_display.insert(tk.END, "Logging stopped.\n")
        self.log_display.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLoggerApp(root)
    root.mainloop()
