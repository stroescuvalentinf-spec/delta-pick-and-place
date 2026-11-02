import threading
import time
import numpy as np
import serial
import serial.tools.list_ports
import tkinter.messagebox as tkmb


class SerialManager:
    def __init__(self, main_win):
        self.win = main_win
        self.ser = None
        self.running = False
        self.thread = None
        self.homed = False
        self.expecting_limits = False

    def connect_signals(self):
        self.win.btn_refresh_ports.configure(command=self.refresh_ports)
        self.win.btn_connect.configure(command=self.on_connect_clicked)
        self.win.btn_homing.configure(command=self.on_homing)
        self.win.btn_upload_params.configure(command=self.on_upload_params)
        self.win.btn_test_limits.configure(command=self.on_test_limits)
        # Init ports on load
        self.refresh_ports()

    def refresh_ports(self):
        self.win.cmb_ports.configure(values=[])
        ports = serial.tools.list_ports.comports()
        vals = [p.device for p in ports]
        if not vals: vals = ["None"]
        self.win.cmb_ports.configure(values=vals)
        self.win.cmb_ports.set(vals[0])

    def on_connect_clicked(self):
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.win.cmb_ports.get()
        if not port or port == "None":
            tkmb.showerror("Serial", "No port selected")
            return
        # Try-Catch connection
        try:
            baud = int(self.win.cmb_baud.get())
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            self.win.btn_connect.configure(text="DISCONNECT", fg_color="red")
            self.win.update_status_bar()
        except Exception as e:
            tkmb.showerror("Connection Failed", str(e))
            self.ser = None

    def disconnect_serial(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None
        self.win.btn_connect.configure(text="CONNECT", fg_color="#2ECC71")
        self.win.update_status_bar()

    def _reader_loop(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line: self.win.after(0, lambda l=line: self.on_serial_line(l))
                else:
                    time.sleep(0.01)
            except:
                break

    def on_serial_line(self, line):
        # Update logs if viewing settings (optional)
        if hasattr(self.win, 'arduino_log'):
            self.win.arduino_log.insert("end", f"< {line}\n")
            self.win.arduino_log.see("end")
        if "HOME DONE" in line.upper():
            self.homed = True
            self.win.ee_pos = np.array([0.0, 0.0, -280.0], dtype=float)
            self.win.update_param_fields()
            tkmb.showinfo("Homing", "Robot Homing Complete!")
        # Handle Limits Test Response
        if self.expecting_limits:
            line_lower = line.lower()
            # Ignore basic command echoes, show the actual response body
            if line_lower != "ok" and "m119" not in line_lower:
                tkmb.showinfo("Limits Report", f"Limit switch states:\n{line}")
                self.expecting_limits = False

    def serial_write(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                if hasattr(self.win, 'arduino_log'):
                    self.win.arduino_log.insert("end", f"> {cmd}\n")
                    self.win.arduino_log.see("end")
                return True
            except Exception as e:
                print(f"Serial Write Error: {e}")
        return False

    def on_homing(self):
        if not self.serial_write("G28"):
            tkmb.showerror("Serial", "Not Connected!")

    def on_upload_params(self):
        if not (self.ser and self.ser.is_open):
            tkmb.showerror("Serial", "Not Connected! Connect first.")
            return
        try:
            # Gather values
            steps = self.win.steps_spin.get().strip()
            micro = self.win.micro_spin.get().strip()
            accel = self.win.accel_spin.get().strip()
            curr = self.win.current_spin.get().strip()
            ratio = self.win.gear_ratio_spin.get().strip()
            # Check emptiness
            if not all([steps, micro, accel, curr, ratio]):
                raise ValueError("All fields must be filled.")
            accel_int = max(100, int(round(float(accel))))
            # Send G-codes
            cmds = [
                f"M350 {micro}",  # Microsteps
                f"M204 A{accel_int}",  # Acceleration
                f"M906 {curr}",  # Current
                f"M92 R{float(ratio):.2f}"  # Ratio
            ]
            for c in cmds:
                self.serial_write(c)
                time.sleep(0.05)  # Small delay between sends
            self.win._last_sent_accel = accel_int
            tkmb.showinfo("Upload", "Parameters uploaded successfully.")
        except ValueError as ve:
            tkmb.showerror("Invalid Input", f"Error: {ve}\nPlease check values.")
        except Exception as e:
            tkmb.showerror("Error", f"Failed to upload: {e}")

    def on_test_limits(self):
        if not self.serial_write("M119"):
            tkmb.showerror("Serial", "Not Connected!")
        else:
            self.expecting_limits = True
            # Timeout after 2 seconds if no response
            self.win.after(2000, self._cancel_expecting_limits)

    def _cancel_expecting_limits(self):
        if self.expecting_limits:
            self.expecting_limits = False
            tkmb.showinfo("Limits Report", "No response received within 2 seconds.")