import tkinter as tk
from tkinter import font as tkfont
import time
import lcm
import sys

from cassie_status_lcm_backend import (
    PDPublisher,
    HeightPublisher,
    StatusSubscriber
)


class CassieStatusApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Cassie Status Panel")
        self.geometry("200x300")

        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.pd_pub = PDPublisher(self.lc)
        self.height_pub = HeightPublisher(self.lc)
        self.status_sub = StatusSubscriber(self.lc, "INPUT_SUPERVISOR_STATUS")

        self.create_widgets()

        self.after(50, self.update_status)

    def create_widgets(self):
        self.status_label = tk.Label(self, text="", font=("Helvetica", 12), justify=tk.LEFT, anchor="nw")
        self.status_label.pack(pady=20, padx=20, fill=tk.BOTH, expand=True)

        self.height_var = tk.StringVar(value="0.90")
        self.height_entry = tk.Entry(self, textvariable=self.height_var, font=("Helvetica", 16))
        self.height_entry.pack(pady=10)

        self.set_height_button = tk.Button(self, text="Set Height", command=self.set_height, font=("Helvetica", 14))
        self.set_height_button.pack(pady=10)

        self.publish_button = tk.Button(self, text="Publish", command=self.publish, font=("Helvetica", 14))
        self.publish_button.pack(pady=10)

    def set_height(self):
        try:
            h = float(self.height_var.get())
            self.height_pub.publish(h)
        except ValueError:
            print("Invalid height value")

    def publish(self):
        self.pd_pub.publish()

    def update_status(self):
        status = self.status_sub.status_text
        self.status_label.config(text=status)
        self.lc.handle_timeout(5)
        self.after(50, self.update_status)


if __name__ == "__main__":
    app = CassieStatusApp()
    app.mainloop()
