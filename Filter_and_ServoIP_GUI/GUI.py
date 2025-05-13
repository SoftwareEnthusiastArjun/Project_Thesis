import socket
import threading
import tkinter as tk
from tkinter import ttk, messagebox

ESP32_HOST = "esp32.local"  # Use IP like "192.168.x.x" if mDNS fails
ESP32_PORT = 12345

class FilterGUI:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Filter Control")
        root.geometry("450x400")

        # Tabs
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(expand=1, fill="both")

        # Tab 1: Filter Control
        self.control_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.control_tab, text="Filter Control")
        self.build_control_tab(self.control_tab)

        # Tab 2: Servo Visualization
        self.servo_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.servo_tab, text="Servo Input")
        self.build_servo_tab(self.servo_tab)

        self.client = None
        self.pwm_stream_thread_started = False

    def build_control_tab(self, tab):
        connection_frame = ttk.LabelFrame(tab, text="Connection", padding=10)
        connection_frame.pack(fill="x", padx=10, pady=5)

        self.status_label = ttk.Label(connection_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side="left")

        connect_btn = ttk.Button(connection_frame, text="Connect", command=self.connect)
        connect_btn.pack(side="right")

        controls_frame = ttk.LabelFrame(tab, text="Filter Parameters", padding=10)
        controls_frame.pack(fill="both", padx=10, pady=5, expand=True)

        self.sliders = {}
        self.slider_vars = {}
        filters = [
            ('ACCEL_FILTER', 0, 1),
            ('GYRO_FILTER', 0, 1),
            ('COMP_FILTER', 0, 1)
        ]

        for idx, (name, min_val, max_val) in enumerate(filters):
            ttk.Label(controls_frame, text=name).grid(row=idx, column=0, sticky="w", pady=2)

            self.slider_vars[name] = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(
                controls_frame,
                from_=min_val,
                to=max_val,
                variable=self.slider_vars[name],
                command=lambda val, n=name: self.on_slider_change(n, val),
                length=200
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)

            value_label = ttk.Label(controls_frame, text="0.000", width=6)
            value_label.grid(row=idx, column=2, padx=5, pady=2)

            self.sliders[name] = {
                'slider': slider,
                'label': value_label
            }

        actions_frame = ttk.Frame(tab, padding=10)
        actions_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(actions_frame, text="Read Current Values", command=self.read_values).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Save to EEPROM", command=self.save_values).pack(side="right", padx=5)

    def build_servo_tab(self, tab):
        servo_frame = ttk.LabelFrame(tab, text="Servo Signal Input", padding=10)
        servo_frame.pack(padx=20, pady=30, fill="both", expand=True)

        self.pwm_canvas = tk.Canvas(servo_frame, height=30, width=300, bg="white", bd=1, relief="sunken")
        self.pwm_canvas.pack(pady=10)

        self.pwm_bar = self.pwm_canvas.create_rectangle(0, 0, 0, 30, fill="green")
        self.pwm_text = ttk.Label(servo_frame, text="PWM: 0%", font=("Arial", 12))
        self.pwm_text.pack()

        # Start streaming PWM when servo tab is opened
        self.tabs.bind("<<NotebookTabChanged>>", self.on_tab_change)

    def connect(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(3)
            self.client.connect((ESP32_HOST, ESP32_PORT))
            self.status_label.config(text="Status: Connected", foreground="green")
            self.read_values()
        except Exception as e:
            self.status_label.config(text="Status: Connection failed", foreground="red")
            messagebox.showerror("Connection Error", str(e))
            self.client = None

    def send_command(self, command):
        try:
            if not self.client:
                self.connect()
                if not self.client:
                    return ""
            self.client.sendall((command + "\n").encode())
            response = self.client.recv(1024).decode().strip()
            return response
        except Exception as e:
            self.status_label.config(text="Status: Error", foreground="red")
            messagebox.showerror("Communication Error", str(e))
            self.client = None
            return ""

    def on_slider_change(self, name, value):
        val = float(value)
        self.sliders[name]['label'].config(text=f"{val:.3f}")

    def read_values(self):
        response = self.send_command("get")
        try:
            a, g, c = map(float, response.split(','))

            self.slider_vars['ACCEL_FILTER'].set(a)
            self.sliders['ACCEL_FILTER']['label'].config(text=f"{a:.3f}")

            self.slider_vars['GYRO_FILTER'].set(g)
            self.sliders['GYRO_FILTER']['label'].config(text=f"{g:.3f}")

            self.slider_vars['COMP_FILTER'].set(c)
            self.sliders['COMP_FILTER']['label'].config(text=f"{c:.3f}")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}")

    def save_values(self):
        a = self.slider_vars['ACCEL_FILTER'].get()
        g = self.slider_vars['GYRO_FILTER'].get()
        c = self.slider_vars['COMP_FILTER'].get()

        self.send_command(f"setA{a:.3f}")
        self.send_command(f"setG{g:.3f}")
        self.send_command(f"setC{c:.3f}")

        response = self.send_command("save")
        if "OK" in response:
            messagebox.showinfo("EEPROM", "Values saved to EEPROM.")
        else:
            messagebox.showerror("EEPROM", "Failed to save values.")

    def on_tab_change(self, event):
        if self.tabs.index(self.tabs.select()) == 1 and not self.pwm_stream_thread_started:
            self.pwm_stream_thread_started = True
            self.start_pwm_stream_thread()

    def start_pwm_stream_thread(self):
        if not self.client:
            self.connect()
            if not self.client:
                return

        def stream_pwm():
            try:
                self.client.sendall(b"startPWMStream\n")
                while True:
                    data = self.client.recv(64)
                    if not data:
                        break
                    lines = data.decode().strip().split('\n')
                    for line in lines:
                        try:
                            if "No signal" in line:
                                percent = 0
                            else:
                                percent = int(line.strip())
                                percent = max(0, min(100, percent))
                            self.root.after(0, self.update_pwm_bar_display, percent)
                        except ValueError:
                            continue
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("PWM Stream Error", str(e)))

        threading.Thread(target=stream_pwm, daemon=True).start()

    def update_pwm_bar_display(self, percent):
        self.pwm_canvas.coords(self.pwm_bar, 0, 0, 3 * percent, 30)
        self.pwm_text.config(text=f"PWM: {percent}%")

if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use('clam')
    app = FilterGUI(root)
    root.mainloop()
