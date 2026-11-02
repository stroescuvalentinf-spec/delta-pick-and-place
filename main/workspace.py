import numpy as np
import threading
import customtkinter as ctk
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from kinematics import delta_calcForward, delta_calcInverse


class Worker(threading.Thread):
    def __init__(self, e, f, re, rf, min_alpha, max_alpha, n_ang, callback):
        super().__init__()
        self.e, self.f, self.re, self.rf = e, f, re, rf
        self.min_alpha, self.max_alpha = min_alpha, max_alpha
        self.n_ang = n_ang
        self.callback = callback
        self._is_running = True
        self.daemon = True

    def run(self):
        angles = np.linspace(-90.0, 90.0, self.n_ang)
        pts = []
        for a1 in angles:
            if not self._is_running: break
            for a2 in angles:
                if not self._is_running: break
                for a3 in angles:
                    if not self._is_running: break
                    s_fw, x, y, z = delta_calcForward(a1, a2, a3, self.e, self.f, self.re, self.rf)
                    if s_fw == 0 and np.isfinite(x):
                        # Reverse check
                        s_ik, _, _, _ = delta_calcInverse(x, y, z, self.e, self.f, self.re, self.rf,
                                                          self.min_alpha, self.max_alpha)
                        if s_ik == 0:
                            pts.append((x, y, z))

        if self._is_running:
            if pts:
                arr = np.array(pts)
                self.callback(arr[:, 0], arr[:, 1], arr[:, 2])
            else:
                self.callback(np.array([]), np.array([]), np.array([]))

    def stop(self):
        self._is_running = False


class WorkspaceWindow(ctk.CTkToplevel):
    def __init__(self, e, f, re, rf, min_alpha, max_alpha, parent=None, n_ang=15):
        super().__init__(parent)
        self.title("Workspace 3D Scatter Plot")
        self.geometry("800x600")

        self.lbl_status = ctk.CTkLabel(self, text="Computing Workspace... (Please Wait)", text_color="orange")
        self.lbl_status.pack(pady=5)

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.btn_close = ctk.CTkButton(self, text="Close", command=self.destroy)
        self.btn_close.pack(pady=5)

        self.worker = Worker(e, f, re, rf, min_alpha, max_alpha, n_ang, self.on_finished)
        self.worker.start()

    def on_finished(self, X, Y, Z):
        # Schedule GUI update on main thread
        self.after(0, lambda: self._plot(X, Y, Z))

    def _plot(self, X, Y, Z):
        self.lbl_status.pack_forget()
        self.ax.clear()

        if X.size == 0:
            self.ax.text2D(0.5, 0.5, "No reachable points.", transform=self.ax.transAxes)
        else:
            sc = self.ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=2, alpha=0.5)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")

            # Auto scale
            max_range = np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max() / 2.0
            mid_x = (X.max() + X.min()) * 0.5
            mid_y = (Y.max() + Y.min()) * 0.5
            mid_z = (Z.max() + Z.min()) * 0.5
            self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax.set_zlim(mid_z - max_range, mid_z + max_range)

        self.canvas.draw()

    def destroy(self):
        if hasattr(self, 'worker'): self.worker.stop()
        super().destroy()