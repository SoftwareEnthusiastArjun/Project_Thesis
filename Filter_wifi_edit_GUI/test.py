import socket
import tkinter as tk
from tkinter import ttk, messagebox

ESP32_HOST = "esp32.local"  # Or use "192.168.x.x" if mDNS doesn't work
ESP32_PORT = 12345

class FilterGUI:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Filter Control")
        
        # Make window slightly larger
        root.geometry("400x350")
        
        # Connection frame
        self.connection_frame = ttk.LabelFrame(root, text="Connection", padding=10)
        self.connection_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        # Connection status
        self.status_label = ttk.Label(
            self.connection_frame, 
            text="Status: Disconnected", 
            foreground="red"
        )
        self.status_label.pack(side="left")
        
        # Connect button
        self.connect_btn = ttk.Button(
            self.connection_frame, 
            text="Connect", 
            command=self.connect
        )
        self.connect_btn.pack(side="right")
        
        # Filter controls frame
        self.controls_frame = ttk.LabelFrame(root, text="Filter Parameters", padding=10)
        self.controls_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        # Sliders
        self.sliders = {}
        self.slider_vars = {}
        filters = [
            ('ACCEL_FILTER', 0, 1),
            ('GYRO_FILTER', 0, 1),
            ('COMP_FILTER', 0, 1)
        ]
        
        for idx, (name, min_val, max_val) in enumerate(filters):
            # Label
            label = ttk.Label(self.controls_frame, text=name)
            label.grid(row=idx, column=0, sticky="w", pady=2)
            
            # Slider
            self.slider_vars[name] = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(
                self.controls_frame,
                from_=min_val,
                to=max_val,
                variable=self.slider_vars[name],
                command=lambda val, n=name: self.on_slider_change(n, val),
                length=200
            )
            slider.grid(row=idx, column=1, padx=5, pady=2)
            
            # Value display
            value_label = ttk.Label(self.controls_frame, text="0.000", width=6)
            value_label.grid(row=idx, column=2, padx=5, pady=2)
            
            self.sliders[name] = {
                'slider': slider,
                'label': value_label
            }
        
        # Action buttons frame
        self.actions_frame = ttk.Frame(root, padding=10)
        self.actions_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        
        # Buttons
        ttk.Button(
            self.actions_frame,
            text="Read Current Values",
            command=self.read_values
        ).pack(side="left", padx=5)
        
        ttk.Button(
            self.actions_frame,
            text="Save to EEPROM",
            command=self.save_values
        ).pack(side="right", padx=5)
        
        self.client = None

    def connect(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(3)
            self.client.connect((ESP32_HOST, ESP32_PORT))
            self.status_label.config(text="Status: Connected", foreground="green")
            self.connect_btn.config(state="disabled")
            self.read_values()  # Auto-read on connection
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
            print("ESP32 ->", response)
            self.status_label.config(text="Status: Connected", foreground="green")
            return response
        except Exception as e:
            self.status_label.config(text="Status: Error", foreground="red")
            messagebox.showerror("Communication Error", str(e))
            self.client = None
            return ""

    def on_slider_change(self, name, value):
        # Update the value label
        val = float(value)
        self.sliders[name]['label'].config(text=f"{val:.3f}")
        
        # Optional: Send updates to ESP32 in real-time
        # self.send_command(f"set{name[0]}{val:.3f}")

    def read_values(self):
        response = self.send_command("get")
        try:
            a, g, c = map(float, response.split(','))
            
            # Update sliders and labels
            self.slider_vars['ACCEL_FILTER'].set(a)
            self.sliders['ACCEL_FILTER']['label'].config(text=f"{a:.3f}")
            
            self.slider_vars['GYRO_FILTER'].set(g)
            self.sliders['GYRO_FILTER']['label'].config(text=f"{g:.3f}")
            
            self.slider_vars['COMP_FILTER'].set(c)
            self.sliders['COMP_FILTER']['label'].config(text=f"{c:.3f}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Invalid response from ESP32: {e}")

    def save_values(self):
        # Get current values from sliders
        a = self.slider_vars['ACCEL_FILTER'].get()
        g = self.slider_vars['GYRO_FILTER'].get()
        c = self.slider_vars['COMP_FILTER'].get()
        
        # Send updates to ESP32
        self.send_command(f"setA{a:.3f}")
        self.send_command(f"setG{g:.3f}")
        self.send_command(f"setC{c:.3f}")
        
        # Save to EEPROM
        response = self.send_command("save")
        if "OK" in response:
            messagebox.showinfo("EEPROM", "Values saved to EEPROM.")
        else:
            messagebox.showerror("EEPROM", "Failed to save values.")

if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use('clam')  # Optional: Makes the UI look more modern
    gui = FilterGUI(root)
    root.mainloop()