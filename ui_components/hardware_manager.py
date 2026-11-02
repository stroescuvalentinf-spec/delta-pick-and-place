import threading
import time
import tkinter.messagebox as tkmb

_HAS_GPIO = False
_IMPORT_ERROR = ""

# Attempt to import gpiozero and the specific factory for Pi 5
try:
    from gpiozero import Button, Device, OutputDevice
    from gpiozero.pins.lgpio import LGPIOFactory

    Device.pin_factory = LGPIOFactory()
    _HAS_GPIO = True

except ImportError as e:
    _HAS_GPIO = False
    _IMPORT_ERROR = str(e)
    print(f"[Hardware] Import Error: {e}")
except Exception as e:
    _HAS_GPIO = False
    _IMPORT_ERROR = str(e)
    print(f"[Hardware] Setup Error: {e}")


class HardwareManager:
    def __init__(self, main_win):
        self.win = main_win
        self.mode_switch = None
        self.estop_button = None

        self.tool_relay = None
        self.green_light_relay = None

        self.running = True
        self.gpio_available = _HAS_GPIO

        self.is_manual_mode = True
        self.estop_active = False

        if self.gpio_available:
            try:
                self.mode_switch = Button(12, pull_up=True)
                self.estop_button = Button(16, pull_up=True)

                # Configure Tool Relay on GPIO23
                self.tool_relay = OutputDevice(23, active_high=False, initial_value=False)

                # Configure Green Light Relay on GPIO24
                self.green_light_relay = OutputDevice(24, active_high=False, initial_value=False)

                self.thread = threading.Thread(target=self._gpio_loop, daemon=True)
                self.thread.start()
                print("[Hardware] GPIO initialized on pins 12, 16, 23 (Tool), and 24 (Green Light).")

            except Exception as e:
                print(f"[Hardware] GPIO Init Failed: {e}")
                self.gpio_available = False
                self.win.after(1000, lambda: tkmb.showwarning("Hardware Error", f"GPIO Init Failed.\n{e}"))
        else:
            print(f"[Hardware] GPIO library missing or failed. Error: {_IMPORT_ERROR}")
            print("Running in Mock Mode.")
            self.win.after(1000, lambda: tkmb.showwarning("Hardware Error",
                                                          f"GPIO Library Error:\n{_IMPORT_ERROR}\n\nRunning in Mock Mode."))

    def _gpio_loop(self):
        while self.running:
            try:
                if self.mode_switch and not getattr(self.mode_switch, 'closed', True):
                    is_manual_read = self.mode_switch.is_pressed

                    if self.is_manual_mode != is_manual_read:
                        self.is_manual_mode = is_manual_read
                        self.win.after(0, self.win.update_status_bar)

                if self.estop_button and not getattr(self.estop_button, 'closed', True):
                    current_estop_state = not self.estop_button.is_pressed

                    if self.estop_active != current_estop_state:
                        self.estop_active = current_estop_state
                        self.win.after(0, lambda s=current_estop_state: self.win.handle_estop(s))

            except Exception as e:
                if not self.running:
                    break

            time.sleep(0.1)

    def set_tool_state(self, state):
        if not self.gpio_available:
            return
        if self.tool_relay and not getattr(self.tool_relay, 'closed', True):
            if state:
                self.tool_relay.on()  # Pulls GPIO 23 to GND
            else:
                self.tool_relay.off()  # Releases GPIO 23 to HIGH

    def set_green_light(self, state):
        if not self.gpio_available:
            return
        if self.green_light_relay and not getattr(self.green_light_relay, 'closed', True):
            if state:
                self.green_light_relay.on()  # Pulls GPIO 24 to GND
            else:
                self.green_light_relay.off()  # Releases GPIO 24 to HIGH

    def stop(self):
        self.running = False
        try:
            if self.mode_switch:
                self.mode_switch.close()
            if self.estop_button:
                self.estop_button.close()
            if self.tool_relay:
                self.tool_relay.off()
                self.tool_relay.close()
            if self.green_light_relay:
                self.green_light_relay.off()
                self.green_light_relay.close()
        except:
            pass