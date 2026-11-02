import sys
import customtkinter as ctk

# Ensure we can import from ui_components if it's a sibling in the root
sys.path.append("..")
from ui_components.main_window import DeltaVisualizer


def main():
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    app = DeltaVisualizer()
    app.mainloop()


if __name__ == "__main__":
    main()