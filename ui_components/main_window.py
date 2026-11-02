import json
import os
import shutil
import numpy as np
import customtkinter as ctk
import tkinter.messagebox as tkmb

from ui_components.ui_builder import UiBuilder
from ui_components.serial_manager import SerialManager
from ui_components.event_handlers import EventHandlers
from ui_components.vision_manager import VisionManager
from ui_components.hardware_manager import HardwareManager

CONFIG_FILE = "config.json"


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NumpyEncoder, self).default(obj)


class TkTimer:
    def __init__(self, widget, callback):
        self.widget = widget
        self.callback = callback
        self._job = None

    def start(self, ms):
        self.stop()
        self._job = self.widget.after(ms, self.callback)

    def stop(self):
        if self._job:
            try:
                self.widget.after_cancel(self._job)
            except:
                pass
            self._job = None


class DeltaVisualizer(ctk.CTk, EventHandlers):
    def __init__(self):
        super().__init__()
        self.title("FANUC Style Teach Pendant")
        self.attributes('-fullscreen', True)
        self.configure(fg_color="#1C1C1C")
        self.e = 70.0
        self.f = 171.0
        self.re = 280.0
        self.rf = 115.0
        self.min_upper_arm_angle = -60.0
        self.max_upper_arm_angle = 80.0
        self.dt = 0.02
        self.ee_pos = np.array([0.0, 0.0, -280.0], dtype=float)
        self.offsets = {'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'tool': 0.0}

        # TEACHING IDE playback state
        self.move_steps = []
        self.step_index = 0
        self.playback_timer = TkTimer(self, self.on_playback_timer)
        self._last_sent_accel = None

        # Isolated HMI Pick & Place playback state
        self.hmi_move_steps = []
        self.hmi_step_index = 0
        self.hmi_playback_timer = TkTimer(self, self.on_hmi_playback_timer)
        self.ignored_close_objects = []
        self.last_picked_belt_ref = None

        self._jog_repeat_timers = {}
        self.hmi_running = False
        self.hmi_state = "IDLE"
        self.hmi_ok_parts = 0
        self.hmi_nok_parts = 0
        self.hmi_stats = {}
        self.hmi_start_time = 0
        self.recipes = {"PickAndPlace": {}}
        self.conveyor_speed_mm_s = 0.0
        self.ui_builder = UiBuilder(self)
        self.ui_builder.build_ui()
        self.serial_manager = SerialManager(self)
        self.vision_manager = VisionManager(self)
        self.ui_builder.connect_signals()
        self.serial_manager.connect_signals()
        self.vision_manager.build_ui_connects()
        for txt, (btn, vec) in self.ui_builder.win.jog_buttons.items():
            btn.bind("<ButtonPress-1>", lambda e, v=vec: self.start_jog_repeat(v))
            btn.bind("<ButtonRelease-1>", lambda e, v=vec: self.stop_jog_repeat(v))
        self.load_config()
        self.update_param_fields()
        self.hardware_manager = HardwareManager(self)
        self.update_status_bar()
        self.hmi_overlay.place_forget()

    def update_status_bar(self):
        ready = "READY" if (self.serial_manager.ser and self.serial_manager.ser.is_open) else "NOT READY"
        r_col = "#2ECC71" if ready == "READY" else "#E74C3C"
        mode = "MANUAL" if (hasattr(self, 'hardware_manager') and self.hardware_manager.is_manual_mode) else "AUTO"
        m_col = "#F39C12" if mode == "MANUAL" else "#3498DB"
        if hasattr(self.ui_builder, 'status_ready_lbl'):
            self.ui_builder.status_ready_lbl.configure(text=ready, text_color=r_col)
            self.ui_builder.status_mode_lbl.configure(text=mode, text_color=m_col)

    def handle_estop(self, is_active):
        if not hasattr(self.ui_builder, 'estop_overlay'):
            return
        if is_active:
            self.ui_builder.estop_overlay.place(relx=0, rely=0, relwidth=1, relheight=1)
            self.ui_builder.estop_overlay.tkraise()
            if self.serial_manager.ser:
                self.serial_manager.serial_write("STOP")
            self.playback_timer.stop()
            self.hmi_playback_timer.stop()
            self.on_tool_toggle(False, sync_ui=True)
            self.on_hmi_stop()
        else:
            self.ui_builder.estop_overlay.place_forget()

    def open_hmi_overlay(self):
        # Stop any active TEACHING IDE program and turn off tool when switching to HMI mode
        self.on_stop()
        self.hmi_overlay.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.hmi_overlay.tkraise()

    def close_hmi_overlay(self):
        self.on_hmi_stop()
        self.hmi_overlay.place_forget()

    def load_config(self):
        def set_text(w, t):
            if w is not None:
                w.delete(0, "end")
                w.insert(0, str(t))

        self.home_coords_loaded = {'x': 0.0, 'y': 0.0, 'z': -280.0}
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    conf = json.load(f)
                offs = conf.get('offsets', {})
                for k in self.offsets.keys():
                    if k in offs:
                        self.offsets[k] = float(offs[k])
                geom = conf.get('geometry', {})
                self.e = float(geom.get('e', self.e))
                self.f = float(geom.get('f', self.f))
                self.re = float(geom.get('re', self.re))
                self.rf = float(geom.get('rf', self.rf))
                self.home_coords_loaded = conf.get('home', self.home_coords_loaded)
                self.recipes = conf.get('recipes', {"PickAndPlace": {}})
                self.conveyor_speed_mm_s = float(conf.get('conveyor_speed_mm_s', 0.0))
                vm = conf.get('vision_matrix')
                if vm:
                    self.vision_manager.transform_matrix = np.array(vm, dtype=np.float32)
                    if hasattr(self, 'lbl_calib_info'):
                        self.lbl_calib_info.configure(text="Matrix Loaded from config. Ready for Test Mode.")
                cam_offs = conf.get('camera_offset')
                if cam_offs:
                    self.vision_manager.camera_offset = cam_offs
                test_rec = conf.get('test_recipe')
                if test_rec:
                    self.vision_manager.test_recipe = test_rec
                roi = conf.get('vision_roi')
                if roi and len(roi) == 4:
                    self.vision_manager.roi = [int(x) for x in roi]
                if hasattr(self, 'cmb_hmi_recipe') and self.cmb_hmi_recipe.winfo_exists():
                    self.cmb_hmi_recipe.configure(values=list(self.recipes.keys()))
                    if list(self.recipes.keys()):
                        self.cmb_hmi_recipe.set(list(self.recipes.keys())[0])
            except Exception as e:
                print(f"Error loading config: {e}\nCorrupted file detected. Loading safe defaults.")
                try:
                    shutil.copy(CONFIG_FILE, CONFIG_FILE + ".bak")
                except:
                    pass
        set_text(self.joff1_edit, f"{self.offsets.get('j1', 0.0):.3f}")
        set_text(self.joff2_edit, f"{self.offsets.get('j2', 0.0):.3f}")
        set_text(self.joff3_edit, f"{self.offsets.get('j3', 0.0):.3f}")
        set_text(self.xoff_edit, f"{self.offsets.get('x', 0.0):.3f}")
        set_text(self.yoff_edit, f"{self.offsets.get('y', 0.0):.3f}")
        set_text(self.zoff_edit, f"{self.offsets.get('z', 0.0):.3f}")
        set_text(self.tool_offset_edit, f"{self.offsets.get('tool', 0.0):.3f}")
        set_text(self.e_edit, self.e)
        set_text(self.f_edit, self.f)
        set_text(self.re_edit, self.re)
        set_text(self.rf_edit, self.rf)
        if hasattr(self, 'base_to_floor_edit'):
            set_text(self.base_to_floor_edit, "370.0")
        set_text(self.home_x, self.home_coords_loaded.get('x', 0.0))
        set_text(self.home_y, self.home_coords_loaded.get('y', 0.0))
        set_text(self.home_z, self.home_coords_loaded.get('z', -280.0))

    def save_config(self):
        try:
            home_coords = {'x': float(self.home_x.get()), 'y': float(self.home_y.get()), 'z': float(self.home_z.get())}
            conf = {
                'offsets': self.offsets.copy(),
                'base_to_floor': float(self.base_to_floor_edit.get()) if self.base_to_floor_edit.get() else 370.0,
                'limits': {'min_angle': float(self.min_upper_arm_angle), 'max_angle': float(self.max_upper_arm_angle)},
                'home': home_coords,
                'geometry': {'e': float(self.e), 'f': float(self.f), 're': float(self.re), 'rf': float(self.rf)},
                'recipes': self.recipes,
                'conveyor_speed_mm_s': float(self.conveyor_speed_mm_s)
            }
            if self.vision_manager.transform_matrix is not None:
                conf['vision_matrix'] = self.vision_manager.transform_matrix.tolist()
            conf['camera_offset'] = getattr(self.vision_manager, 'camera_offset', [0, 0])
            conf['test_recipe'] = getattr(self.vision_manager, 'test_recipe', {})
            if hasattr(self.vision_manager, 'roi') and self.vision_manager.roi is not None:
                conf['vision_roi'] = self.vision_manager.roi
            temp_file = CONFIG_FILE + ".tmp"
            with open(temp_file, 'w') as f:
                json.dump(conf, f, indent=2, cls=NumpyEncoder)
            os.replace(temp_file, CONFIG_FILE)
            if hasattr(self, 'cmb_hmi_recipe') and self.cmb_hmi_recipe.winfo_exists():
                self.cmb_hmi_recipe.configure(values=list(self.recipes.keys()))
        except Exception as e:
            tkmb.showerror("Error", str(e))

    def on_closing(self):
        if hasattr(self, 'hardware_manager'):
            self.hardware_manager.stop()
        self.vision_manager.stop()
        if self.serial_manager.ser:
            self.serial_manager.disconnect_serial()
        self.destroy()