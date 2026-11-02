import glob
import customtkinter as ctk
from .recorder import RecorderPanel
import tkinter.messagebox as tkmb

COLOR_BG = "#1C1C1C"
COLOR_PANEL = "#2D2D30"
COLOR_HEADER = "#000000"
COLOR_YELLOW = "#FFD700"
COLOR_YELLOW_DARK = "#F4D03F"
COLOR_TEXT_W = "#FFFFFF"
COLOR_DANGER = "#E74C3C"
COLOR_SUCCESS = "#2ECC71"


class DataEntry(ctk.CTkEntry):
    def __init__(self, master, **kwargs):
        cfg = {"fg_color": "#333333", "border_color": "gray", "text_color": "white", "justify": "center",
               "font": ("Consolas", 18), "height": 40}
        cfg.update(kwargs)
        super().__init__(master, **cfg)


class FanucButton(ctk.CTkButton):
    def __init__(self, master, text, **kwargs):
        cfg = {"fg_color": COLOR_YELLOW_DARK, "hover_color": "#D4AC0D", "text_color": "black", "corner_radius": 4,
               "height": 55, "font": ("Roboto", 16, "bold")}
        cfg.update(kwargs)
        super().__init__(master, text=text, **cfg)


class UiBuilder:
    def __init__(self, main_win):
        self.win = main_win

    def build_ui(self):
        self.win.grid_rowconfigure(0, weight=0)
        self.win.grid_rowconfigure(1, weight=1)
        self.win.grid_columnconfigure(0, weight=0)
        self.win.grid_columnconfigure(1, weight=1)
        status_bar = ctk.CTkFrame(self.win, height=60, fg_color=COLOR_HEADER, corner_radius=0)
        status_bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        status_bar.grid_propagate(False)
        ctk.CTkLabel(status_bar, text="SYSTEM STATUS:", font=("Arial", 20, "bold"), text_color="white").pack(
            side="left", padx=(20, 10))
        self.status_ready_lbl = ctk.CTkLabel(status_bar, text="NOT READY", font=("Arial", 20, "bold"),
                                             text_color=COLOR_DANGER)
        self.status_ready_lbl.pack(side="left", padx=10)
        ctk.CTkLabel(status_bar, text="|  MODE:", font=("Arial", 20, "bold"), text_color="white").pack(side="left",
                                                                                                       padx=(30, 10))
        self.status_mode_lbl = ctk.CTkLabel(status_bar, text="MANUAL", font=("Arial", 20, "bold"),
                                            text_color=COLOR_YELLOW)
        self.status_mode_lbl.pack(side="left", padx=10)
        ctk.CTkButton(status_bar, text="X", width=50, height=40, fg_color=COLOR_DANGER, font=("Arial", 16, "bold"),
                      command=self.win.on_closing).pack(side="right", padx=10)
        ctk.CTkButton(status_bar, text="_", width=50, height=40, fg_color="#555555", font=("Arial", 16, "bold"),
                      command=self.win.iconify).pack(side="right", padx=5)
        nav_panel = ctk.CTkFrame(self.win, width=280, corner_radius=0, fg_color=COLOR_PANEL)
        nav_panel.grid(row=1, column=0, sticky="nsew")
        nav_panel.grid_propagate(False)
        ctk.CTkLabel(nav_panel, text="TEACH\nPENDANT", font=("Arial", 24, "bold"), text_color=COLOR_YELLOW).pack(
            pady=30)
        self.frames = {}
        pages = ["Controls", "Jog", "Teaching", "Parameters", "Vision", "Settings", "HMI"]
        for name in pages:
            if name == "HMI":
                cmd = lambda: self.win.open_hmi_overlay()
            else:
                cmd = lambda n=name: self.select_frame(n)
            ctk.CTkButton(nav_panel, text=name.upper(), command=cmd, height=60, fg_color="#444444",
                          hover_color="#666666", font=("Arial", 16, "bold"), corner_radius=5).pack(fill="x", padx=10,
                                                                                                   pady=5)
        self.stack_frame = ctk.CTkFrame(self.win, fg_color=COLOR_BG, corner_radius=0)
        self.stack_frame.grid(row=1, column=1, sticky="nsew")
        self.stack_frame.grid_rowconfigure(0, weight=1)
        self.stack_frame.grid_columnconfigure(0, weight=1)
        for name in pages:
            if name != "HMI":
                f = ctk.CTkFrame(self.stack_frame, fg_color="transparent")
                self.frames[name] = f
                f.grid(row=0, column=0, sticky="nsew")
        self._create_controls_page(self.frames["Controls"])
        self._create_jog_page(self.frames["Jog"])
        self._create_recorder_page(self.frames["Teaching"])
        self._create_parameters_page(self.frames["Parameters"])
        self._create_vision_page(self.frames["Vision"])
        self._create_comm_page(self.frames["Settings"])
        self._create_hmi_overlay()
        self._create_sliders_dummy(self.win)
        self.select_frame("Controls")
        self.estop_overlay = ctk.CTkFrame(self.win, fg_color=COLOR_DANGER, corner_radius=0)
        c = ctk.CTkFrame(self.estop_overlay, fg_color="transparent")
        c.place(relx=0.5, rely=0.5, anchor="center")
        ctk.CTkLabel(c, text="  EMERGENCY STOP  ", font=("Arial", 90, "bold"), text_color="white").pack(pady=20)
        ctk.CTkLabel(c, text="SYSTEM HALTED", font=("Arial", 50, "bold"), text_color="yellow").pack(pady=20)

    def select_frame(self, name):
        self.frames[name].tkraise()

    def connect_signals(self):
        self.win.btn_apply.configure(command=self.win.on_apply_target)
        self.win.btn_home.configure(command=self.win.on_home_send)
        self.win.btn_stop.configure(command=self.win.on_stop)
        self.win.recorder_widget.registered.connect(self.win.on_recorder_register)
        self.win.recorder_widget.played.connect(self.win.on_recorder_play)
        self.win.recorder_widget.pause_toggled.connect(self.win.on_recorder_pause_toggled)
        self.win.recorder_widget.stepped.connect(self.win.on_recorder_step)
        self.win.recorder_widget.tool_toggled.connect(self.win.on_tool_toggle)
        self.win.btn_generate_anim.configure(command=self.win.on_generate_animation)
        self.win.btn_test_limits.configure(command=self.win.serial_manager.on_test_limits)
        self.win.btn_homing.configure(command=self.win.serial_manager.on_homing)
        if hasattr(self.win, 'jog_vel_spin'):
            self.win.jog_vel_spin.bind("<KeyRelease>", lambda e: self.win.on_jog_vel_changed(e, apply_hw=False))
            self.win.jog_vel_spin.bind("<FocusOut>", lambda e: self.win.on_jog_vel_changed(e, apply_hw=False))
            self.win.jog_vel_spin.bind("<Return>", lambda e: self.win.on_jog_vel_changed(e, apply_hw=False))
        if hasattr(self.win, 'btn_apply_offsets'):
            self.win.btn_apply_offsets.configure(command=self.win.apply_offsets)
        if hasattr(self.win, 'btn_save_offsets'):
            self.win.btn_save_offsets.configure(command=self.win.save_offsets)
        if hasattr(self.win, 'btn_reset_offsets'):
            self.win.btn_reset_offsets.configure(command=self.win.reset_offsets)

    def _create_sliders_dummy(self, parent):
        self.win.slider_x = ctk.CTkSlider(parent, width=0, height=40, button_length=40);
        self.win.spin_x = ctk.CTkEntry(parent)
        self.win.slider_y = ctk.CTkSlider(parent, width=0, height=40, button_length=40);
        self.win.spin_y = ctk.CTkEntry(parent)
        self.win.slider_z = ctk.CTkSlider(parent, width=0, height=40, button_length=40);
        self.win.spin_z = ctk.CTkEntry(parent)
        self.win.lbl_x_range = ctk.CTkLabel(parent, text="");
        self.win.lbl_y_range = ctk.CTkLabel(parent, text="");
        self.win.lbl_z_range = ctk.CTkLabel(parent, text="")

    def _create_hmi_overlay(self):
        self.win.hmi_overlay = ctk.CTkFrame(self.win, fg_color=COLOR_BG, corner_radius=0)
        hdr = ctk.CTkFrame(self.win.hmi_overlay, fg_color=COLOR_HEADER, height=80, corner_radius=0)
        hdr.pack(fill="x")
        ctk.CTkLabel(hdr, text="PRODUCTION HMI", font=("Arial", 28, "bold"), text_color=COLOR_YELLOW).pack(side="left",
                                                                                                           padx=30,
                                                                                                           pady=20)
        ctk.CTkButton(hdr, text="CLOSE HMI", fg_color="#555555", font=("Arial", 16, "bold"), width=150, height=50,
                      command=self.win.close_hmi_overlay).pack(side="right", padx=20, pady=15)
        content = ctk.CTkFrame(self.win.hmi_overlay, fg_color="transparent")
        content.pack(fill="both", expand=True, padx=40, pady=40)
        left_col = ctk.CTkFrame(content, fg_color=COLOR_PANEL, corner_radius=10)
        left_col.pack(side="left", fill="y", expand=True, padx=20)
        right_col = ctk.CTkFrame(content, fg_color=COLOR_PANEL, corner_radius=10, width=400)
        right_col.pack(side="right", fill="y", padx=20)
        right_col.pack_propagate(False)
        ctk.CTkLabel(left_col, text="SYSTEM CONTROLS", font=("Arial", 24, "bold")).pack(pady=(30, 10))
        self.win.btn_hmi_start = ctk.CTkButton(left_col, text="  START PROCESS", width=400, height=100,
                                               fg_color=COLOR_SUCCESS, font=("Arial", 32, "bold"),
                                               command=self.win.on_hmi_start)
        self.win.btn_hmi_start.pack(pady=15)
        self.win.btn_hmi_stop = ctk.CTkButton(left_col, text="  STOP PROCESS", width=400, height=100,
                                              fg_color=COLOR_DANGER, font=("Arial", 32, "bold"),
                                              command=self.win.on_hmi_stop)
        self.win.btn_hmi_stop.pack(pady=15)
        ctk.CTkLabel(left_col, text="RECIPE MANAGEMENT", font=("Arial", 20, "bold"), text_color="gray").pack(
            pady=(30, 10))
        self.win.cmb_hmi_recipe = ctk.CTkOptionMenu(left_col, values=["PickAndPlace"], width=300, height=50,
                                                    font=("Arial", 18, "bold"))
        self.win.cmb_hmi_recipe.pack(pady=10)
        btn_row = ctk.CTkFrame(left_col, fg_color="transparent")
        btn_row.pack(pady=10)
        self.win.btn_hmi_edit = ctk.CTkButton(btn_row, text="EDIT RECIPE", width=190, height=55, fg_color="#3498DB",
                                              font=("Arial", 16, "bold"), command=self.win.on_hmi_edit_recipe)
        self.win.btn_hmi_edit.pack(side="left", padx=10)
        self.win.btn_conveyor_calib = ctk.CTkButton(btn_row, text="CALIB SPEED", width=190, height=55,
                                                    fg_color="#E67E22", font=("Arial", 16, "bold"),
                                                    command=self.win.on_calibrate_conveyor)
        self.win.btn_conveyor_calib.pack(side="left", padx=10)
        ctk.CTkLabel(right_col, text="PRODUCTION STATS", font=("Arial", 24, "bold"), text_color=COLOR_YELLOW).pack(
            pady=(30, 20))
        self.win.stats_container = ctk.CTkScrollableFrame(right_col, fg_color="transparent")
        try:
            self.win.stats_container._scrollbar.configure(width=30)
        except Exception:
            pass
        self.win.stats_container.pack(fill="both", expand=True, padx=10, pady=10)
        self.win.lbl_hmi_state = ctk.CTkLabel(right_col, text="STATE: IDLE", font=("Arial", 22, "bold"),
                                              text_color="white")
        self.win.lbl_hmi_state.pack(side="bottom", pady=30)

    def _create_controls_page(self, parent):
        ctk.CTkLabel(parent, text="CURRENT COORDINATES", font=("Arial", 22, "bold"), text_color="gray").pack(
            pady=(40, 10))
        readout_frame = ctk.CTkFrame(parent, fg_color=COLOR_PANEL, corner_radius=10)
        readout_frame.pack(fill="x", padx=100, pady=10)
        for axis in ["X", "Y", "Z"]:
            row = ctk.CTkFrame(readout_frame, fg_color="transparent")
            row.pack(fill="x", padx=50, pady=15)
            ctk.CTkLabel(row, text=f"{axis}:", font=("Arial", 36, "bold"), width=60, text_color="white").pack(
                side="left")
            entry = DataEntry(row, width=200, font=("Consolas", 36), height=50)
            entry.pack(side="right", fill="x", expand=True)
            setattr(self.win, f"{axis.lower()}_edit", entry)
        btn_frame = ctk.CTkFrame(parent, fg_color="transparent")
        btn_frame.pack(pady=50)
        self.win.btn_apply = FanucButton(btn_frame, text="MOVE TO TARGET", width=220, height=80, fg_color=COLOR_SUCCESS)
        self.win.btn_apply.pack(side="left", padx=15)
        self.win.btn_home = FanucButton(btn_frame, text="HOME ROBOT", width=220, height=80, fg_color="#2980B9")
        self.win.btn_home.pack(side="left", padx=15)
        self.win.btn_stop = FanucButton(btn_frame, text="STOP", width=220, height=80, fg_color=COLOR_DANGER)
        self.win.btn_stop.pack(side="left", padx=15)
        foot = ctk.CTkFrame(parent, fg_color="transparent")
        foot.pack(side="bottom", fill="x", pady=20, padx=50)
        self.win.lbl_ee = ctk.CTkLabel(foot, text="Coords: --", font=("Consolas", 18))
        self.win.lbl_ee.pack(side="left")
        for v in ["e", "f", "re", "rf", "joff1", "joff2", "joff3", "xoff", "yoff", "zoff", "tool_offset",
                  "base_to_floor"]:
            setattr(self.win, f"{v}_edit", ctk.CTkEntry(parent, width=0, height=0))
        self.win.home_x = ctk.CTkEntry(parent);
        self.win.home_x.insert(0, "0")
        self.win.home_y = ctk.CTkEntry(parent);
        self.win.home_y.insert(0, "0")
        self.win.home_z = ctk.CTkEntry(parent);
        self.win.home_z.insert(0, "-280")
        self.win.simulate_chk = ctk.CTkCheckBox(parent)
        self.win.btn_jog_simulate = ctk.CTkCheckBox(parent)
        self.win.lbl_j1 = ctk.CTkLabel(parent, text="");
        self.win.lbl_j2 = ctk.CTkLabel(parent, text="");
        self.win.lbl_j3 = ctk.CTkLabel(parent, text="")

    def _create_jog_page(self, parent):
        grid = ctk.CTkFrame(parent, fg_color="transparent")
        grid.place(relx=0.5, rely=0.45, anchor="center")
        dirs = [("Y+", (0, 1, 0)), ("Z+", (0, 0, 1)), ("X-", (-1, 0, 0)), ("X+", (1, 0, 0)), ("Y-", (0, -1, 0)),
                ("Z-", (0, 0, -1))]
        btn_pos = [(0, 1), (0, 3), (1, 0), (1, 2), (2, 1), (2, 3)]
        self.win.jog_buttons = {}
        for (txt, vec), (r, c) in zip(dirs, btn_pos):
            b = FanucButton(grid, text=txt, width=140, height=120, font=("Arial", 28, "bold"))
            b.grid(row=r, column=c, padx=20, pady=20)
            self.win.jog_buttons[txt] = (b, vec)
        spd_frame = ctk.CTkFrame(parent, fg_color="transparent")
        spd_frame.place(relx=0.5, rely=0.85, anchor="center")
        ctk.CTkLabel(spd_frame, text="JOG VELOCITY:", font=("Arial", 22, "bold")).pack(side="left", padx=10)
        self.win.jog_vel_spin = DataEntry(spd_frame, width=150)
        self.win.jog_vel_spin.insert(0, "490")
        self.win.jog_vel_spin.pack(side="left", padx=10)
        ctk.CTkLabel(spd_frame, text="mm/s", font=("Arial", 16)).pack(side="left")
        self.win.dt_edit = ctk.CTkEntry(parent);
        self.win.dt_edit.insert(0, "0.02")
        self.win.jog_mode_combo = ctk.CTkOptionMenu(parent);
        self.win.btn_realtime_graph = ctk.CTkButton(parent);
        self.win.chk_collision_jog = ctk.CTkCheckBox(parent)

    def _create_recorder_page(self, parent):
        self.win.recorder_widget = RecorderPanel(parent, embedded=True)
        self.win.recorder_widget.pack(fill="both", expand=True, padx=20, pady=(20, 5))
        anim_frame = ctk.CTkFrame(parent, fg_color=COLOR_PANEL, height=100)
        anim_frame.pack(fill="x", padx=20, pady=20)
        anim_frame.pack_propagate(False)
        ctk.CTkLabel(anim_frame, text="ANIMATION:", font=("Arial", 18, "bold")).pack(side="left", padx=20)
        self.win.animation_combo = ctk.CTkOptionMenu(anim_frame, values=["Spiral", "Circle", "Square"], width=220,
                                                     height=50, font=("Arial", 18, "bold"),
                                                     dropdown_font=("Arial", 18, "bold"), fg_color=COLOR_YELLOW_DARK,
                                                     text_color="black")
        self.win.animation_combo.pack(side="left", padx=10)
        self.win.btn_generate_anim = FanucButton(anim_frame, text="GENERATE PATH", width=220, height=50,
                                                 font=("Arial", 18, "bold"))
        self.win.btn_generate_anim.pack(side="left", padx=10)
        self.win.generate_mode_combo = ctk.CTkOptionMenu(parent);
        self.win.btn_workspace = ctk.CTkButton(parent);
        self.win.chk_collision = ctk.CTkCheckBox(parent);
        self.win.chk_auto_home = ctk.CTkCheckBox(parent)

    def _create_parameters_page(self, parent):
        scroll = ctk.CTkScrollableFrame(parent, fg_color="transparent")
        try:
            scroll._scrollbar.configure(width=30)
        except Exception:
            pass
        scroll.pack(fill="both", expand=True, padx=20, pady=20)
        ctk.CTkLabel(scroll, text="ROBOT DIMENSIONS & SETTINGS", font=("Arial", 20, "bold"),
                     text_color=COLOR_YELLOW).pack(pady=10)
        grid = ctk.CTkFrame(scroll, fg_color=COLOR_PANEL)
        grid.pack(fill="x", pady=10)
        grid.grid_columnconfigure((0, 2), weight=0)
        grid.grid_columnconfigure((1, 3), weight=1)

        def add_row(lbl_text, var_name, r, c):
            ctk.CTkLabel(grid, text=lbl_text, font=("Arial", 16, "bold"), anchor="e").grid(row=r, column=c, padx=20,
                                                                                           pady=10, sticky="e")
            e = DataEntry(grid, width=150)
            e.grid(row=r, column=c + 1, padx=20, pady=10, sticky="w")
            setattr(self.win, var_name, e)

        ctk.CTkLabel(grid, text="GEOMETRY (mm)", font=("Arial", 14, "bold"), text_color="gray").grid(row=0, column=0,
                                                                                                     columnspan=2,
                                                                                                     pady=(10, 5))
        add_row("Effector (E):", "e_edit", 1, 0)
        add_row("Base (F):", "f_edit", 2, 0)
        add_row("Arm Length (RE):", "re_edit", 3, 0)
        add_row("Arm Length (RF):", "rf_edit", 4, 0)
        add_row("Base Height:", "base_to_floor_edit", 5, 0)
        ctk.CTkLabel(grid, text="POS OFFSETS", font=("Arial", 14, "bold"), text_color="gray").grid(row=6, column=0,
                                                                                                   columnspan=2,
                                                                                                   pady=(20, 5))
        add_row("Pos X Offset:", "xoff_edit", 7, 0)
        add_row("Pos Y Offset:", "yoff_edit", 8, 0)
        add_row("Pos Z Offset:", "zoff_edit", 9, 0)
        ctk.CTkLabel(grid, text="JOINT OFFSETS", font=("Arial", 14, "bold"), text_color="gray").grid(row=0, column=2,
                                                                                                     columnspan=2,
                                                                                                     pady=(10, 5))
        add_row("Joint 1 Offset:", "joff1_edit", 1, 2)
        add_row("Joint 2 Offset:", "joff2_edit", 2, 2)
        add_row("Joint 3 Offset:", "joff3_edit", 3, 2)
        add_row("Tool Z Offset:", "tool_offset_edit", 4, 2)
        ctk.CTkLabel(grid, text="HOME COORDS", font=("Arial", 14, "bold"), text_color="gray").grid(row=6, column=2,
                                                                                                   columnspan=2,
                                                                                                   pady=(20, 5))
        add_row("Home X:", "home_x", 7, 2)
        add_row("Home Y:", "home_y", 8, 2)
        add_row("Home Z:", "home_z", 9, 2)
        btn_row = ctk.CTkFrame(parent, fg_color="transparent")
        btn_row.pack(pady=20)
        self.win.btn_apply_offsets = FanucButton(btn_row, text="APPLY", width=120, fg_color=COLOR_SUCCESS)
        self.win.btn_apply_offsets.pack(side="left", padx=10)
        self.win.btn_save_offsets = FanucButton(btn_row, text="SAVE CONFIG", width=120)
        self.win.btn_save_offsets.pack(side="left", padx=10)
        self.win.btn_reset_offsets = FanucButton(btn_row, text="RESET", width=120, fg_color="gray")
        self.win.btn_reset_offsets.pack(side="left", padx=10)

    def _create_vision_page(self, parent):
        scroll_wrapper = ctk.CTkScrollableFrame(parent, fg_color="transparent")
        try:
            scroll_wrapper._scrollbar.configure(width=30)
        except Exception:
            pass
        scroll_wrapper.pack(fill="both", expand=True)
        top = ctk.CTkFrame(scroll_wrapper, fg_color="transparent")
        top.pack(fill="x", padx=20, pady=10)
        devs = glob.glob("/dev/video*")
        valid_cams = sorted(devs) if devs else ["0", "1", "2"]
        self.win.cmb_camera_id = ctk.CTkOptionMenu(top, values=valid_cams, width=150)
        if valid_cams:
            self.win.cmb_camera_id.set(valid_cams[0])
        self.win.cmb_camera_id.pack(side="left", padx=10)
        self.win.btn_camera_toggle = FanucButton(top, text="START CAMERA STREAM", width=250)
        self.win.btn_camera_toggle.pack(side="left", padx=10)
        self.win.lbl_vision_status = ctk.CTkLabel(top, text="Status: OFF", font=("Arial", 16))
        self.win.lbl_vision_status.pack(side="left", padx=20)
        main_content = ctk.CTkFrame(scroll_wrapper, fg_color="transparent")
        main_content.pack(fill="both", expand=True, padx=20, pady=10)
        video_frame = ctk.CTkFrame(main_content, fg_color="transparent")
        video_frame.pack(side="left", fill="both", expand=True)
        self.win.lbl_video_feed = ctk.CTkLabel(video_frame, text="NO SIGNAL", width=640, height=480, fg_color="black")
        self.win.lbl_video_feed.pack(pady=10)
        self.win.lbl_calib_info = ctk.CTkLabel(video_frame,
                                               text="Mode: CALIBRATION | Tip: Jog to X+, X-, Y+, Y-, click EE and Add Point.",
                                               font=("Arial", 16, "bold"), text_color=COLOR_YELLOW)
        self.win.lbl_calib_info.pack(pady=10)
        tools = ctk.CTkFrame(main_content, fg_color=COLOR_PANEL, corner_radius=10)
        tools.pack(side="right", fill="y", padx=20, pady=10)
        coord_frm = ctk.CTkFrame(tools, fg_color="transparent")
        coord_frm.pack(fill="x", padx=15, pady=(20, 10))
        ctk.CTkLabel(coord_frm, text="Robot X:", font=("Arial", 16, "bold")).grid(row=0, column=0, padx=5, pady=10,
                                                                                  sticky="e")
        self.win.txt_calib_x = ctk.CTkEntry(coord_frm, width=100, height=40, font=("Consolas", 18))
        self.win.txt_calib_x.grid(row=0, column=1, padx=5, pady=10)
        ctk.CTkLabel(coord_frm, text="Robot Y:", font=("Arial", 16, "bold")).grid(row=1, column=0, padx=5, pady=10,
                                                                                  sticky="e")
        self.win.txt_calib_y = ctk.CTkEntry(coord_frm, width=100, height=40, font=("Consolas", 18))
        self.win.txt_calib_y.grid(row=1, column=1, padx=5, pady=10)
        btn_grid = ctk.CTkFrame(tools, fg_color="transparent")
        btn_grid.pack(fill="both", expand=True, padx=15, pady=10)
        bw, bh = 140, 90
        fnt = ("Arial", 14, "bold")
        self.win.btn_calib_add = ctk.CTkButton(btn_grid, text="Add Point\n(0/4)", width=bw, height=bh, font=fnt)
        self.win.btn_calib_add.grid(row=0, column=0, padx=10, pady=8)
        self.win.btn_calib_compute = ctk.CTkButton(btn_grid, text="Compute\nMatrix", fg_color=COLOR_SUCCESS, width=bw,
                                                   height=bh, font=fnt)
        self.win.btn_calib_compute.grid(row=0, column=1, padx=10, pady=8)
        self.win.btn_calib_reset = ctk.CTkButton(btn_grid, text="Reset\nCalib", fg_color=COLOR_DANGER, width=bw,
                                                 height=bh, font=fnt)
        self.win.btn_calib_reset.grid(row=1, column=0, padx=10, pady=8)
        self.win.btn_teach_test = ctk.CTkButton(btn_grid, text="Teach\nTarget", fg_color="#F39C12", width=bw, height=bh,
                                                font=fnt)
        self.win.btn_teach_test.grid(row=1, column=1, padx=10, pady=8)
        self.win.btn_test_mode = ctk.CTkButton(btn_grid, text="ENABLE\nTEST MODE", fg_color="#3498DB", width=bw,
                                               height=bh, font=fnt)
        self.win.btn_test_mode.grid(row=2, column=0, padx=10, pady=8)
        self.win.btn_test_move = ctk.CTkButton(btn_grid, text="MOVE TO\nTARGET", fg_color=COLOR_SUCCESS, width=bw,
                                               height=bh, font=fnt)
        self.win.btn_test_move.grid(row=2, column=1, padx=10, pady=8)
        # Added Conveyor ROI controls
        self.win.btn_set_roi = ctk.CTkButton(btn_grid, text="Draw\nROI Area", fg_color="#8E44AD", width=bw, height=bh, font=fnt)
        self.win.btn_set_roi.grid(row=3, column=0, padx=10, pady=8)
        self.win.btn_reset_roi = ctk.CTkButton(btn_grid, text="Reset\nROI Area", fg_color="#C0392B", width=bw, height=bh, font=fnt)
        self.win.btn_reset_roi.grid(row=3, column=1, padx=10, pady=8)

    def _create_comm_page(self, parent):
        ctk.CTkLabel(parent, text="CONNECTION & SETTINGS", font=("Arial", 24, "bold"), text_color=COLOR_YELLOW).pack(
            pady=30)
        conn = ctk.CTkFrame(parent, fg_color=COLOR_PANEL)
        conn.pack(fill="x", padx=100, pady=10)
        self.win.cmb_ports = ctk.CTkOptionMenu(conn, values=["COM1", "/dev/ttyACM0"], width=250, height=50,
                                               font=("Arial", 16))
        self.win.cmb_ports.pack(side="left", padx=30, pady=30)
        self.win.cmb_baud = ctk.CTkOptionMenu(conn, values=["115200", "250000"], width=150, height=50,
                                              font=("Arial", 16))
        self.win.cmb_baud.pack(side="left", padx=10)
        self.win.btn_connect = FanucButton(conn, text="CONNECT", fg_color=COLOR_SUCCESS)
        self.win.btn_connect.pack(side="right", padx=30)
        self.win.btn_refresh_ports = FanucButton(conn, text="REFRESH")
        self.win.btn_refresh_ports.pack(side="right", padx=10)
        params = ctk.CTkFrame(parent, fg_color="transparent")
        params.pack(fill="x", padx=50, pady=20)

        def add_p(txt, var, val):
            row = ctk.CTkFrame(params, fg_color="transparent")
            row.pack(fill="x", pady=5)
            ctk.CTkLabel(row, text=txt, width=200, anchor="e", font=("Arial", 14)).pack(side="left", padx=10)
            e = DataEntry(row, width=300)
            e.insert(0, str(val))
            e.pack(side="left")
            setattr(self.win, var, e)

        add_p("Steps/Rev:", "steps_spin", 200)
        add_p("Microsteps:", "micro_spin", 16)
        add_p("Accel (mm/s ):", "accel_spin", round(490 * 104.55))
        add_p("Current (mA):", "current_spin", 2200)
        add_p("Gear Ratio:", "gear_ratio_spin", 5.0)
        util = ctk.CTkFrame(parent, fg_color="transparent")
        util.pack(pady=20)
        self.win.btn_upload_params = FanucButton(util, text="UPLOAD PARAMS")
        self.win.btn_upload_params.pack(side="left", padx=20)
        self.win.btn_homing = FanucButton(util, text="PERFORM HOMING")
        self.win.btn_homing.pack(side="left", padx=20)
        self.win.btn_test_limits = FanucButton(util, text="TEST LIMITS")
        self.win.btn_test_limits.pack(side="left", padx=20)
        self.win.cmb_limit_mode = ctk.CTkOptionMenu(parent);
        self.win.btn_set_limit_mode = ctk.CTkButton(parent);
        self.win.btn_steps_info = ctk.CTkButton(parent);
        self.win.arduino_log = ctk.CTkTextbox(parent)