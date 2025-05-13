import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time

class FloatValueGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Float Value Controller")
        self.root.geometry("400x300")

        self.esp_ip = "192.168.1.184"
        self.esp_port = 12345
        self.connected = False
        self.client_socket = None
        self.current_value = 0.5

        self.create_widgets()
        self.start_connection_thread()

    def create_widgets(self):
        # Status
        status_frame = ttk.Frame(self.root, padding="10")
        status_frame.pack(fill=tk.X)
        self.status_label = ttk.Label(status_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT)

        # Value display
        value_frame = ttk.Frame(self.root, padding="10")
        value_frame.pack(fill=tk.X)
        ttk.Label(value_frame, text="Current Value:").pack(side=tk.LEFT)
        self.value_display = ttk.Label(value_frame, text="0.50")
        self.value_display.pack(side=tk.LEFT, padx=10)

        # Slider
        slider_frame = ttk.Frame(self.root, padding="10")
        slider_frame.pack(fill=tk.X)
        self.slider = ttk.Scale(slider_frame, from_=0, to=100, value=50, command=self.on_slider_move)
        self.slider.pack(fill=tk.X)
        self.slider_value = ttk.Label(slider_frame, text="0.50")
        self.slider_value.pack()

        # Buttons
        button_frame = ttk.Frame(self.root, padding="10")
        button_frame.pack(fill=tk.X)
        self.set_btn = ttk.Button(button_frame, text="Set Value", command=self.send_value, state=tk.DISABLED)
        self.set_btn.pack(side=tk.LEFT, padx=5)
        self.save_btn = ttk.Button(button_frame, text="Save to EEPROM", command=self.save_to_eeprom, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=5)
        self.refresh_btn = ttk.Button(button_frame, text="Refresh", command=self.get_current_value)
        self.refresh_btn.pack(side=tk.RIGHT, padx=5)

    def on_slider_move(self, value):
        float_value = float(value) / 100
        self.slider_value.config(text=f"{float_value:.2f}")

    def start_connection_thread(self):
        threading.Thread(target=self.connection_loop, daemon=True).start()

    def connection_loop(self):
        while True:
            if not self.connected:
                self.connect_to_esp32()
            else:
                try:
                    self.client_socket.sendall(b"get\n")
                    response = self.client_socket.recv(1024).decode().strip()
                    if not response:
                        raise Exception("No response")
                except:
                    self.connected = False
                    self.client_socket.close()
                    self.client_socket = None
                    self.update_status(False)
            time.sleep(2)

    def connect_to_esp32(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            sock.connect((self.esp_ip, self.esp_port))
            self.client_socket = sock
            self.connected = True
            self.update_status(True)
            self.get_current_value()
        except Exception as e:
            print(f"Connection failed: {e}")
            self.update_status(False)

    def update_status(self, connected):
        def update_ui():
            self.status_label.config(text="Connected" if connected else "Disconnected",
                                     foreground="green" if connected else "red")
            self.set_btn.config(state=tk.NORMAL if connected else tk.DISABLED)
            self.save_btn.config(state=tk.NORMAL if connected else tk.DISABLED)
        self.root.after(0, update_ui)

    def get_current_value(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to ESP32")
            return
        try:
            self.client_socket.sendall(b"get\n")
            response = self.client_socket.recv(1024).decode().strip()
            value = float(response)
            self.current_value = value
            self.root.after(0, lambda: self.value_display.config(text=f"{value:.2f}"))
            self.root.after(0, lambda: self.slider.set(value * 100))
            self.root.after(0, lambda: self.slider_value.config(text=f"{value:.2f}"))
        except Exception as e:
            messagebox.showerror("Error", f"Failed to get value: {e}")

    def send_value(self):
        if not self.connected:
            return  # Silently ignore if disconnected

        value = float(self.slider.get()) / 100
        try:
            self.client_socket.sendall(f"set{value:.2f}\n".encode())
            response = self.client_socket.recv(1024).decode().strip()

            if response == "OK":
                self.current_value = value
                self.value_display.config(text=f"{value:.2f}")
                # Optional: flash a status label or console log instead of popup
                print(f"Value set to {value:.2f}")
            else:
                print(f"Unexpected response: {response}")  # Just log it
        except Exception as e:
            print(f"Error setting value: {e}")

    def save_to_eeprom(self):
        if not self.connected:
            return

        try:
            self.client_socket.sendall(b"save\n")
            response = self.client_socket.recv(1024).decode().strip()

            if response == "OK":
                print("Value saved to EEPROM")
            else:
                print(f"Unexpected response: {response}")
        except Exception as e:
            print(f"Error saving value: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = FloatValueGUI(root)
    root.mainloop()
