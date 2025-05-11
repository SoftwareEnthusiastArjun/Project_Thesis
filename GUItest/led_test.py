import socket
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, 
                            QPushButton, QLabel, QMessageBox, QSizePolicy)
from PyQt5.QtCore import Qt

class LEDControl(QWidget):
    def __init__(self):
        super().__init__()
        self.sock = None
        self.current_mode = "off"
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("ESP32 LED Control")
        self.resize(400, 300)  # Larger window
        
        layout = QVBoxLayout()
        layout.setSpacing(15)
        
        # Status label
        self.status = QLabel("Status: Disconnected")
        self.status.setStyleSheet("font-size: 16px;")
        self.status.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status)
        
        # Connect button
        self.connect_btn = QPushButton("CONNECT")
        self.connect_btn.setStyleSheet("font-size: 14px; height: 40px;")
        self.connect_btn.clicked.connect(self.connect_esp32)
        layout.addWidget(self.connect_btn)
        
        # Control buttons
        btn_style = "font-size: 16px; height: 50px;"
        self.on_btn = QPushButton("TURN ON")
        self.on_btn.setStyleSheet(btn_style)
        self.on_btn.clicked.connect(lambda: self.send_command("on"))
        
        self.off_btn = QPushButton("TURN OFF")
        self.off_btn.setStyleSheet(btn_style)
        self.off_btn.clicked.connect(lambda: self.send_command("off"))
        
        self.blink_btn = QPushButton("BLINK MODE")
        self.blink_btn.setStyleSheet(btn_style)
        self.blink_btn.clicked.connect(lambda: self.send_command("blink"))
        
        layout.addWidget(self.on_btn)
        layout.addWidget(self.off_btn)
        layout.addWidget(self.blink_btn)
        
        # Save button
        self.save_btn = QPushButton("💾 SAVE CURRENT STATE")
        self.save_btn.setStyleSheet("font-size: 16px; height: 50px; background-color: #FFD700;")
        self.save_btn.clicked.connect(lambda: self.send_command("save"))
        layout.addWidget(self.save_btn)
        
        self.setLayout(layout)
        self.update_buttons(False)
        
    def connect_esp32(self):
        try:
            self.sock = socket.socket()
            self.sock.settimeout(5)
            self.sock.connect(("192.168.211.106", 12345))
            self.status.setText("Status: Connected")
            self.status.setStyleSheet("color: green; font-size: 16px;")
            self.update_buttons(True)
            QMessageBox.information(self, "Success", "Successfully connected to ESP32!")
        except Exception as e:
            self.status.setText("Status: Connection Failed")
            self.status.setStyleSheet("color: red; font-size: 16px;")
            QMessageBox.critical(self, "Error", f"Connection failed:\n{str(e)}")
            
    def send_command(self, cmd):
        if not self.sock:
            QMessageBox.warning(self, "Error", "Not connected to ESP32!")
            return
            
        try:
            self.sock.sendall((cmd + "\n").encode())
            response = self.sock.recv(1024).decode().strip()
            
            if response.startswith("OK_"):
                self.current_mode = response[3:]
                QMessageBox.information(self, "Success", f"LED is now {self.current_mode.upper()}")
            elif response.startswith("SAVED_"):
                QMessageBox.information(self, "Saved", f"State '{response[6:].upper()}' saved to EEPROM!")
            else:
                QMessageBox.warning(self, "Response", f"Unexpected response: {response}")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Command failed:\n{str(e)}")
            self.disconnect()
            
    def update_buttons(self, connected):
        state = connected
        self.on_btn.setEnabled(state)
        self.off_btn.setEnabled(state)
        self.blink_btn.setEnabled(state)
        self.save_btn.setEnabled(state)
        
    def disconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.status.setText("Status: Disconnected")
        self.status.setStyleSheet("color: red; font-size: 16px;")
        self.update_buttons(False)
        
    def closeEvent(self, event):
        self.disconnect()
        event.accept()

if __name__ == "__main__":
    app = QApplication([])
    app.setStyle('Fusion')  # Better looking style
    
    window = LEDControl()
    window.show()
    app.exec_()
    