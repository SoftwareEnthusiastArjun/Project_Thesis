import socket
import threading
from time import sleep
import tkinter as tk
from tkinter import ttk, messagebox
from cube_visualizer2 import CubeVisualizer2  # Import CubeViewer class
import serial
import queue
ESP32_HOST = "esp32.local"  # Use IP like "192.168.x.x" if mDNS fails
ESP32_PORT = 12345

class FilterGUI:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Control Panel")
        root.geometry("700x550")  # Slightly larger window

        self.filter_sliders = {}
        self.filter_vars = {}

        self.pitch_sliders = {}
        self.pitch_vars = {}

        self.roll_sliders = {}
        self.roll_vars = {}

        # Tabs
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(expand=1, fill="both")

        # Existing tabs
        self.control_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.control_tab, text="Filter Control")
        self.build_control_tab(self.control_tab)

        self.servo_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.servo_tab, text="Servo Input")
        self.build_servo_tab(self.servo_tab)

        self.visualizer_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.visualizer_tab, text="3D Visualizer")
        self.build_visualizer_tab(self.visualizer_tab)

        # New PID tabs
        self.pitch_pid_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.pitch_pid_tab, text="Pitch PID")
        self.build_pid_tab(self.pitch_pid_tab, "Pitch")

        self.roll_pid_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.roll_pid_tab, text="Roll PID")
        self.build_pid_tab(self.roll_pid_tab, "Roll")

        self.client = None
        self.pwm_stream_thread_started = False
        self.viewer = None
        self.viewer_thread = None
        self.data_queue = queue.Queue()


    def build_control_tab(self, tab):
        connection_frame = ttk.LabelFrame(tab, text="Connection", padding=10)
        connection_frame.pack(fill="x", padx=10, pady=5)

        self.status_label = ttk.Label(connection_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(side="left")

        connect_btn = ttk.Button(connection_frame, text="Connect", command=self.connect)
        connect_btn.pack(side="right")

        controls_frame = ttk.LabelFrame(tab, text="Filter Parameters", padding=10)
        controls_frame.pack(fill="both", padx=10, pady=5, expand=True)

        filters = [
            ('ACCEL_FILTER', 0, 1),
            ('GYRO_FILTER', 0, 1),
            ('COMP_FILTER', 0, 1)
        ]

        for idx, (name, min_val, max_val) in enumerate(filters):
            ttk.Label(controls_frame, text=name).grid(row=idx, column=0, sticky="w", pady=2)

            self.filter_vars[name] = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(
                controls_frame,
                from_=min_val,
                to=max_val,
                variable=self.filter_vars[name],
                command=lambda val, n=name: self.on_slider_change(n, val),
                length=250
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)

            value_label = ttk.Label(controls_frame, text="0.000", width=8)
            value_label.grid(row=idx, column=2, padx=5, pady=2)

            self.filter_sliders[name] = {
                'slider': slider,
                'label': value_label
            }

        actions_frame = ttk.Frame(tab, padding=10)
        actions_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(actions_frame, text="Read Current Values", command=self.read_values).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Save to EEPROM", command=self.save_values).pack(side="right", padx=5)

    def build_servo_tab(self, tab):
        servo_frame = ttk.LabelFrame(tab, text="Servo Signal Inputs", padding=10)
        servo_frame.pack(padx=20, pady=20, fill="both", expand=True)

        self.pwm_canvases = []
        self.pwm_bars = []
        self.pwm_labels = []

        for i in range(3):
            canvas = tk.Canvas(servo_frame, height=30, width=300, bg="white", bd=1, relief="sunken")
            canvas.pack(pady=10)

            bar = canvas.create_rectangle(0, 0, 0, 30, fill="green")
            label = ttk.Label(servo_frame, text=f"PWM {i+1}: 0%", font=("Arial", 12))
            label.pack()

            self.pwm_canvases.append(canvas)
            self.pwm_bars.append(bar)
            self.pwm_labels.append(label)

        # Autopilot Switch Label
        self.ap_label = ttk.Label(servo_frame, text="Auto Pilot: Unknown", font=("Arial", 12), foreground="gray")
        self.ap_label.pack(pady=10)

        self.tabs.bind("<<NotebookTabChanged>>", self.on_tab_change)

    #-------------------------------------------------------------------------------------------
    def build_pid_tab(self, tab, pid_type):
        connection_frame = ttk.LabelFrame(tab, text="Connection", padding=10)
        connection_frame.pack(fill="x", padx=10, pady=5)

        status_label = ttk.Label(connection_frame, text="Status: Disconnected", foreground="red")
        status_label.pack(side="left")

        connect_btn = ttk.Button(connection_frame, text="Connect", 
                                command=lambda: self.connect(status_label))
        connect_btn.pack(side="right")

        controls_frame = ttk.LabelFrame(tab, text=f"{pid_type} PID Parameters", padding=10)
        controls_frame.pack(fill="both", padx=10, pady=5, expand=True)

        # Use specific dictionaries for each PID type
        if pid_type == "Pitch":
            sliders = self.pitch_sliders
            vars_ = self.pitch_vars
        else:
            sliders = self.roll_sliders
            vars_ = self.roll_vars

        pid_params = [
            ('Kp', 0, 10),
            ('Ki', 0, 5),
            ('Kd', 0, 5)
        ]

        for idx, (name, min_val, max_val) in enumerate(pid_params):
            ttk.Label(controls_frame, text=name).grid(row=idx, column=0, sticky="w", pady=2)

            vars_[f"{pid_type}_{name}"] = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(
                controls_frame,
                from_=min_val,
                to=max_val,
                variable=vars_[f"{pid_type}_{name}"],
                command=lambda val, n=f"{pid_type}_{name}": self.on_slider_change(n, val),
                length=250
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)

            value_label = ttk.Label(controls_frame, text="0.000", width=8)
            value_label.grid(row=idx, column=2, padx=5, pady=2)

            sliders[f"{pid_type}_{name}"] = {
                'slider': slider,
                'label': value_label
            }

        actions_frame = ttk.Frame(tab, padding=10)
        actions_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(actions_frame, text="Read Current Values", 
                command=lambda: self.read_pid_values(pid_type)).pack(side="left", padx=5)
        ttk.Button(actions_frame, text="Save to EEPROM", 
                command=lambda: self.save_pid_values(pid_type)).pack(side="right", padx=5)


    def read_pid_values(self, pid_type):
        response = self.send_command(f"get{pid_type}PID")
        try:
            if response:
                values = response.split(',')
                if len(values) == 3:
                    kp, ki, kd = map(float, values)
                    vars_ = self.pitch_vars if pid_type == "Pitch" else self.roll_vars
                    sliders = self.pitch_sliders if pid_type == "Pitch" else self.roll_sliders

                    vars_[f"{pid_type}_Kp"].set(kp)
                    sliders[f"{pid_type}_Kp"]['label'].config(text=f"{kp:.3f}")
                    vars_[f"{pid_type}_Ki"].set(ki)
                    sliders[f"{pid_type}_Ki"]['label'].config(text=f"{ki:.3f}")
                    vars_[f"{pid_type}_Kd"].set(kd)
                    sliders[f"{pid_type}_Kd"]['label'].config(text=f"{kd:.3f}")
                else:
                    raise ValueError("Invalid number of values received")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}\nResponse: {response}")


    def save_pid_values(self, pid_type):
        vars_ = self.pitch_vars if pid_type == "Pitch" else self.roll_vars

        kp = vars_[f"{pid_type}_Kp"].get()
        ki = vars_[f"{pid_type}_Ki"].get()
        kd = vars_[f"{pid_type}_Kd"].get()

        self.send_command(f"set{pid_type}P{kp:.3f}")
        self.send_command(f"set{pid_type}I{ki:.3f}")
        self.send_command(f"set{pid_type}D{kd:.3f}")

        response = self.send_command("save")
        if "OK" in response:
            messagebox.showinfo("EEPROM", f"{pid_type} PID values saved to EEPROM.")
        else:
            messagebox.showerror("EEPROM", "Failed to save values.")

    #------------------------------------------------------------------------------------------------

    def connect(self, status_label=None):
        try:
            if not self.client or not self.is_connected():
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.settimeout(5)  # Increased timeout
                self.client.connect((ESP32_HOST, ESP32_PORT))
                if status_label:
                    status_label.config(text="Status: Connected", foreground="green")
                else:
                    self.status_label.config(text="Status: Connected", foreground="green")
        except Exception as e:
            if status_label:
                status_label.config(text="Status: Connection failed", foreground="red")
            else:
                self.status_label.config(text="Status: Connection failed", foreground="red")
            messagebox.showerror("Connection Error", str(e))
            if self.client:
                self.client.close()
            self.client = None

    def is_connected(self):
        try:
            if self.client:
                # Try to send a small test command
                self.client.sendall(b"get\n")
                response = self.client.recv(64)
                return bool(response)
        except:
            return False
        return False

    def send_command(self, command):
        try:
            if not self.client or not self.is_connected():
                self.connect()
                if not self.client:
                    return ""
            
            # Clear receive buffer before sending new command
            self.client.setblocking(0)
            try:
                while self.client.recv(1024): pass
            except:
                pass
            self.client.setblocking(1)
            
            self.client.sendall((command + "\n").encode())
            response = self.client.recv(1024).decode().strip()
            
            # Debug print
            print(f"Sent: {command}, Received: {response}")
            
            return response
        except Exception as e:
            messagebox.showerror("Communication Error", str(e))
            if self.client:
                self.client.close()
            self.client = None
            return ""

    def on_slider_change(self, name, value):
        val = float(value)
        for sliders in [self.filter_sliders, self.pitch_sliders, self.roll_sliders]:
            if name in sliders:
                sliders[name]['label'].config(text=f"{val:.3f}")
                return
        print(f"[WARN] Unknown slider name: {name}")



    def read_values(self):
        response = self.send_command("get")
        try:
            if response:
                values = response.split(',')
                if len(values) == 3:
                    a, g, c = map(float, values)
                    self.filter_vars['ACCEL_FILTER'].set(a)
                    self.filter_sliders['ACCEL_FILTER']['label'].config(text=f"{a:.3f}")
                    self.filter_vars['GYRO_FILTER'].set(g)
                    self.filter_sliders['GYRO_FILTER']['label'].config(text=f"{g:.3f}")
                    self.filter_vars['COMP_FILTER'].set(c)
                    self.filter_sliders['COMP_FILTER']['label'].config(text=f"{c:.3f}")
                else:
                    raise ValueError("Invalid number of values received")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}\nResponse: {response}")


    def save_values(self):
        a = self.filter_vars['ACCEL_FILTER'].get()
        g = self.filter_vars['GYRO_FILTER'].get()
        c = self.filter_vars['COMP_FILTER'].get()

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
                        if "No signal" in line:
                            percentages = [0, 0, 0, 0]
                        else:
                            try:
                                percentages = list(map(int, line.strip().split(',')))
                                percentages = [max(0, min(100, p)) for p in percentages]
                                if len(percentages) < 4:
                                    percentages += [0] * (4 - len(percentages))
                            except:
                                continue
                        self.root.after(0, self.update_pwm_bar_display, percentages)
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: messagebox.showerror("PWM Stream Error", error_msg))

        threading.Thread(target=stream_pwm, daemon=True).start()

    def update_pwm_bar_display(self, percentages):
        for i, percent in enumerate(percentages[:3]):
            self.pwm_canvases[i].coords(self.pwm_bars[i], 0, 0, 3 * percent, 30)
            self.pwm_labels[i].config(text=f"PWM {i+1}: {percent}%")

        if len(percentages) >= 4:
            autopilot_percent = percentages[3]
            if 0 <= autopilot_percent <= 10:
                self.ap_label.config(text="Auto Pilot: OFF", foreground="red")
            elif 90 <= autopilot_percent <= 100:
                self.ap_label.config(text="Auto Pilot: ON", foreground="green")
            else:
                self.ap_label.config(text="Auto Pilot: Unknown", foreground="gray")

    def build_visualizer_tab(self, tab):
        vis_frame = ttk.LabelFrame(tab, text="3D Cube Visualizer", padding=10)
        vis_frame.pack(padx=20, pady=20, fill="both", expand=True)

        def start_cube():
            if not self.client:
                self.connect()
                if not self.client:
                    return

            try:
                self.client.sendall(b"startCubeStream\n")
                print("[INFO] Sent 'startCubeStream' command.")
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to send startCubeStream: {e}")
                return

            # Start CubeVisualizer2 thread if not running
            if not self.viewer or not self.viewer.is_alive():
                self.data_queue = queue.Queue()
                self.viewer = CubeVisualizer2(self.data_queue)
                self.viewer.start()

            def cube_stream_loop():
                buffer = ""
                stopped = False
                try:
                    while not stopped:
                        data = self.client.recv(1024)
                        if not data:
                            break

                        buffer += data.decode()
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if not line:
                                continue

                            if line == "CUBE_STREAM_STOPPED":
                                print("[INFO] Cube stream stopped by ESP32.")
                                stopped = True
                                continue

                            if "CUBE_STREAM_START" in line:
                                # Ignore start marker line
                                continue

                            try:
                                gx, gy, gz = map(float, line.split(','))
                                print(f"gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
                                # Put gyro values into the queue for visualizer
                                if self.viewer and self.viewer.is_alive():
                                    self.data_queue.put((gx, gy, gz))
                            except ValueError:
                                print(f"[WARN] Invalid data received: {line}")

                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror("Cube Stream Error", str(e)))

            threading.Thread(target=cube_stream_loop, daemon=True).start()

        def stop_cube():
            if not self.client:
                return

            try:
                self.client.sendall(b"stopCubeStream\n")
                buffer = ""
                timeout_counter = 0

                while timeout_counter < 10:  # Try for up to ~1 second
                    data = self.client.recv(1024)
                    if not data:
                        break
                    buffer += data.decode()
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line == "CUBE_STREAM_STOPPED":
                            print("[INFO] Cube stream stopped on ESP32.")
                            if self.viewer and self.viewer.is_alive():
                                try:
                                    self.viewer.stop()
                                    self.viewer.join(timeout=2)
                                    self.viewer = None
                                    with self.data_queue.mutex:
                                        self.data_queue.queue.clear()
                                except Exception as e:
                                    messagebox.showerror("Visualizer Error", f"Failed to stop cube visualizer: {e}")
                            return  # Exit the function successfully
                        elif line:
                            print(f"[INFO] Ignoring line during stop: {line}")
                    sleep(0.1)
                    timeout_counter += 1

                # If we exit loop without seeing stop confirmation
                messagebox.showerror("Stream Error", "Failed to stop cube stream on ESP32: No confirmation received.")

            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to send stopCubeStream: {e}")

        ttk.Button(vis_frame, text="Start Cube", command=start_cube).pack(pady=10)
        ttk.Button(vis_frame, text="Stop Cube", command=stop_cube).pack(pady=10)




if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use('clam')
    app = FilterGUI(root)
    root.mainloop()