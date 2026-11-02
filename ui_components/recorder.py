import customtkinter as ctk
import re
import tkinter.filedialog


class LineNumbers(ctk.CTkTextbox):
    def __init__(self, master, **kwargs):
        super().__init__(master, activate_scrollbars=False, **kwargs)
        self.configure(state="disabled", fg_color="#2b2b2b", text_color="gray", width=50)

    def update_lines(self, line_count):
        self.configure(state="normal")
        self.delete("0.0", "end")
        content = "\n".join(str(i) for i in range(1, line_count + 1))
        self.insert("0.0", content)
        self.configure(state="disabled")


class RecorderPanel(ctk.CTkFrame):
    def __init__(self, parent, embedded=False):
        super().__init__(parent)
        self.registered = Signals()
        self.played = Signals()
        self.stepped = Signals()
        self.pause_toggled = Signals()
        self.saved = Signals()
        self.trajectory_requested = Signals()
        self.tool_toggled = Signals()

        editor_frame = ctk.CTkFrame(self, fg_color="transparent")
        editor_frame.pack(side="left", fill="both", expand=True)

        # Increased Font Size
        font_config = ("Consolas", 16)
        self.line_numbers = LineNumbers(editor_frame, font=font_config)
        self.line_numbers.pack(side="left", fill="y", padx=(0, 2))

        self.editor = ctk.CTkTextbox(editor_frame, font=font_config, wrap="none")
        self.editor.pack(side="left", fill="both", expand=True)

        self.editor.bind("<KeyRelease>", self.on_content_changed)
        self.editor.bind("<MouseWheel>", self.on_scroll_editor)
        self.editor.bind("<Button-4>", self.on_scroll_editor)
        self.editor.bind("<Button-5>", self.on_scroll_editor)

        self.line_numbers.bind("<MouseWheel>", self.on_scroll_lines)
        self.line_numbers.bind("<Button-4>", self.on_scroll_lines)
        self.line_numbers.bind("<Button-5>", self.on_scroll_lines)

        self.editor.tag_config("highlight", background="#3498DB", foreground="white")

        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(side="right", fill="y", padx=10, pady=5)

        col1 = ctk.CTkFrame(bar, fg_color="transparent")
        col1.pack(side="left", fill="y", padx=(5, 10))
        col2 = ctk.CTkFrame(bar, fg_color="transparent")
        col2.pack(side="left", fill="y", padx=(10, 5))

        btn_cfg = {"height": 60, "width": 160, "font": ("Arial", 16, "bold")}

        ctk.CTkLabel(col1, text="Control", font=("Arial", 18, "bold")).pack(pady=(5, 5))
        self.btn_play = ctk.CTkButton(col1, text="  Play", fg_color="green", command=self.on_play, **btn_cfg)
        self.btn_play.pack(pady=4)
        self.btn_step = ctk.CTkButton(col1, text="Step", command=self.on_step, **btn_cfg)
        self.btn_step.pack(pady=4)
        self.btn_pause = ctk.CTkButton(col1, text="Pause", command=self.on_pause, **btn_cfg)
        self.btn_pause.pack(pady=4)

        # Single Tool checkbox, made bigger for touch screen
        self.btn_tool = ctk.CTkCheckBox(col1, text="Tool", command=self.on_tool,
                                        font=("Arial", 22, "bold"), checkbox_width=45, checkbox_height=45)
        self.btn_tool.pack(pady=(20, 15))

        ctk.CTkLabel(col1, text="File", font=("Arial", 18, "bold")).pack(pady=(10, 5))
        ctk.CTkButton(col1, text="Clear", fg_color="red", command=self.clear, **btn_cfg).pack(pady=4)
        ctk.CTkButton(col1, text="Save", command=self.save_script, **btn_cfg).pack(pady=4)
        ctk.CTkButton(col1, text="Load", command=self.load_script, **btn_cfg).pack(pady=4)

        ctk.CTkLabel(col2, text="Syntax", font=("Arial", 18, "bold")).pack(pady=(5, 5))
        ctk.CTkButton(col2, text="Move(Rec)", command=self.on_move, **btn_cfg).pack(pady=4)
        ctk.CTkButton(col2, text="Delay(1s)", command=lambda: self.insert_text("Delay(1.0)"), **btn_cfg).pack(pady=4)
        ctk.CTkButton(col2, text="Tool(ON)", command=lambda: self.insert_text("Tool(ON)"), **btn_cfg).pack(pady=4)
        ctk.CTkButton(col2, text="Tool(OFF)", command=lambda: self.insert_text("Tool(OFF)"), **btn_cfg).pack(pady=4)
        ctk.CTkButton(col2, text="Loop()", command=lambda: self.insert_text("Loop()"), **btn_cfg).pack(pady=4)

        self.loop_enabled = False
        self._parsed_moves = []
        self.clear()

    def on_content_changed(self, event=None):
        count = int(self.editor._textbox.index("end-1c").split(".")[0])
        self.line_numbers.update_lines(count)
        self.sync_views()

    def on_scroll_editor(self, event=None):
        self.after(10, self.sync_views)

    def on_scroll_lines(self, event=None):
        if event.num == 4 or event.delta > 0:
            self.editor._textbox.yview_scroll(-1, "units")
        elif event.num == 5 or event.delta < 0:
            self.editor._textbox.yview_scroll(1, "units")
        self.sync_views()

    def sync_views(self):
        try:
            y_pos = self.editor._textbox.yview()
            self.line_numbers._textbox.yview_moveto(y_pos[0])
        except Exception:
            pass

    def clear(self):
        self.editor.delete("0.0", "end")
        self.editor.insert("0.0", "Begin()\n\nEnd()")
        self._parsed_moves = []
        self.loop_enabled = False
        self.clear_highlight()
        self.on_content_changed()

    def insert_text(self, txt):
        current = self.editor.get("0.0", "end")
        if "End()" in current or "END()" in current.upper():
            self.editor.insert("end-2l", txt + "\n")
        else:
            self.editor.insert("end", txt + "\n")
        self.on_content_changed()

    def add_move(self, params):
        if 'jog' in params and 'accel' in params:
            cmd = (f"Move({params['x']:.2f}, {params['y']:.2f}, {params['z']:.2f}, "
                   f"{params['jog']:.1f}, {params['accel']:.0f})")
        elif 'jog' in params:
            cmd = f"Move({params['x']:.2f}, {params['y']:.2f}, {params['z']:.2f}, {params['jog']:.1f})"
        else:
            cmd = f"Move({params['x']:.2f}, {params['y']:.2f}, {params['z']:.2f})"
            if 't' in params:
                cmd = cmd[:-1] + f", t={params['t']:.2f})"
        self.insert_text(cmd)

    def on_move(self):
        self.registered.emit(None)

    def on_play(self):
        self.btn_pause.configure(text="Pause")
        if self.parse_script():
            self.played.emit(None)

    def on_step(self):
        if not self._parsed_moves:
            self.parse_script()
        self.stepped.emit(None)

    def on_pause(self):
        txt = self.btn_pause.cget("text")
        new_state = (txt == "Pause")
        self.btn_pause.configure(text="Resume" if new_state else "Pause")
        self.pause_toggled.emit(new_state)

    def reset_pause_button(self):
        self.btn_pause.configure(text="Pause")

    def on_tool(self):
        val = bool(self.btn_tool.get())
        self.tool_toggled.emit(val)

    def set_tool_checkbox(self, state):
        if state:
            self.btn_tool.select()
        else:
            self.btn_tool.deselect()

    def parse_script(self):
        txt = self.editor.get("0.0", "end")
        lines = txt.split('\n')
        parsed = []
        self.loop_enabled = False
        num_pattern = r"[-+]?(?:\d*\.\d+|\d+)"

        for i, line in enumerate(lines):
            line_clean = line.strip()
            if not line_clean or line_clean.startswith("//") or line_clean.startswith("#"):
                continue
            line_upper = line_clean.upper()

            if "LOOP()" in line_upper:
                self.loop_enabled = True

            if line_upper.startswith("MOVE"):
                try:
                    parts = re.findall(num_pattern, line_clean)
                    if len(parts) >= 3:
                        cmd = {
                            'type': 'move',
                            'x': float(parts[0]),
                            'y': float(parts[1]),
                            'z': float(parts[2]),
                            'jog': None,
                            'accel': None,
                            'line': i + 1
                        }
                        if len(parts) >= 4:
                            cmd['jog'] = float(parts[3])
                        if len(parts) >= 5:
                            cmd['accel'] = float(parts[4])
                        parsed.append(cmd)
                except Exception:
                    pass
            elif line_upper.startswith("DELAY"):
                try:
                    parts = re.findall(num_pattern, line_clean)
                    val = float(parts[0]) if parts else 1.0
                    parsed.append({'type': 'delay', 'dt': val, 'line': i + 1})
                except Exception:
                    pass
            elif line_upper.startswith("TOOL"):
                state = 1 if "ON" in line_upper else 0
                parsed.append({'type': 'tool', 'state': state, 'line': i + 1})

        self._parsed_moves = parsed
        self.trajectory_requested.emit(None)
        return True

    def moves(self):
        return self._parsed_moves

    def highlight_line(self, line_num):
        self.clear_highlight()
        if line_num > 0:
            start = f"{line_num}.0"
            end = f"{line_num + 1}.0"
            self.editor.tag_add("highlight", start, end)
            self.editor.see(start)
            self.after(50, self.sync_views)

    def clear_highlight(self):
        self.editor.tag_remove("highlight", "1.0", "end")

    def save_script(self):
        fn = tkinter.filedialog.asksaveasfilename(defaultextension=".txt")
        if fn:
            with open(fn, 'w') as f:
                f.write(self.editor.get("0.0", "end"))
            self.saved.emit(fn)

    def load_script(self):
        fn = tkinter.filedialog.askopenfilename()
        if fn:
            with open(fn, 'r') as f:
                self.editor.delete("0.0", "end")
                self.editor.insert("0.0", f.read())
            self.parse_script()
            self.on_content_changed()


class Signals:
    def __init__(self):
        self.cb = []

    def connect(self, func):
        self.cb.append(func)

    def emit(self, val):
        for c in self.cb:
            try:
                c(val)
            except TypeError:
                c()

    def disconnect(self):
        self.cb = []