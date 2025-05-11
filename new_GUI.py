import sys
import serial
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QSlider, QPushButton,
    QDoubleSpinBox, QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

class StabilizerConfigGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.ser = None
        self.params = {
            'accel_filter': 0.3,
            'gyro_filter': 0.08,
            'comp_filter': 0.7
        }
        
        self.init_serial()
        self.init_ui()
        self.setup_timers()
        
        # Request current parameters after startup
        QTimer.singleShot(1000, self.request_current_params)

    def init_serial(self):
        """Initialize serial connection"""
        try:
            self.ser = serial.Serial('COM8', 38400, timeout=1)  # Change COM port if needed
            print("Serial connection established")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.ser = None

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ESP32 Stabilizer Config")
        self.resize(400, 500)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Create filter controls
        self.accel_group, self.accel_slider, self.accel_spinbox = self.create_filter_control(
            "Accelerometer Filter", self.params['accel_filter'], self.update_accel_filter)
        self.gyro_group, self.gyro_slider, self.gyro_spinbox = self.create_filter_control(
            "Gyroscope Filter", self.params['gyro_filter'], self.update_gyro_filter)
        self.comp_group, self.comp_slider, self.comp_spinbox = self.create_filter_control(
            "Complementary Filter", self.params['comp_filter'], self.update_comp_filter)
        
        layout.addWidget(self.accel_group)
        layout.addWidget(self.gyro_group)
        layout.addWidget(self.comp_group)
        
        # Action buttons
        btn_layout = QHBoxLayout()
        self.calibrate_btn = QPushButton("Calibrate Gyro")
        self.calibrate_btn.clicked.connect(self.send_calibrate)
        
        self.reset_yaw_btn = QPushButton("Reset Yaw")
        self.reset_yaw_btn.clicked.connect(self.send_reset_yaw)
        
        self.save_btn = QPushButton("Save to EEPROM")
        self.save_btn.clicked.connect(self.save_to_eeprom)

        # NEW: Safe Exit Button
        self.exit_btn = QPushButton("Exit")
        self.exit_btn.clicked.connect(self.safe_exit)
        self.exit_btn.setStyleSheet("background-color: #ff4444; color: white;")
        
        btn_layout.addWidget(self.calibrate_btn)
        btn_layout.addWidget(self.reset_yaw_btn)
        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.exit_btn)  # Add Exit button
        
        layout.addLayout(btn_layout)

    def create_filter_control(self, label, value, callback):
        """Create a filter control group"""
        group = QGroupBox(label)
        layout = QVBoxLayout()
        
        slider = QSlider(Qt.Horizontal)
        slider.setRange(1, 100)
        slider.setValue(int(value * 100))
        slider.valueChanged.connect(lambda v: callback(v / 100.0))
        
        spinbox = QDoubleSpinBox()
        spinbox.setRange(0.01, 1.0)
        spinbox.setSingleStep(0.01)
        spinbox.setValue(value)
        spinbox.valueChanged.connect(callback)
        
        layout.addWidget(slider)
        layout.addWidget(spinbox)
        group.setLayout(layout)
        
        return group, slider, spinbox

    def setup_timers(self):
        """Setup timers for periodic tasks"""
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.check_serial_status)
        self.status_timer.start(1000)  # Check every second

    def update_accel_filter(self, value):
        """Update accelerometer filter value"""
        self.params['accel_filter'] = value
        self.accel_spinbox.setValue(value)
        self.accel_slider.setValue(int(value * 100))
        self.send_params()

    def update_gyro_filter(self, value):
        """Update gyroscope filter value"""
        self.params['gyro_filter'] = value
        self.gyro_spinbox.setValue(value)
        self.gyro_slider.setValue(int(value * 100))
        self.send_params()

    def update_comp_filter(self, value):
        """Update complementary filter value"""
        self.params['comp_filter'] = value
        self.comp_spinbox.setValue(value)
        self.comp_slider.setValue(int(value * 100))
        self.send_params()

    def send_params(self):
        """Send current parameters to ESP32"""
        if self.ser and self.ser.is_open:
            try:
                cmd = f"p{self.params['accel_filter']:.4f},{self.params['gyro_filter']:.4f},{self.params['comp_filter']:.4f}\n"
                self.ser.write(cmd.encode())
            except serial.SerialException as e:
                print(f"Error sending parameters: {e}")

    def send_calibrate(self):
        """Send calibration command"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"c\n")
            except serial.SerialException as e:
                print(f"Error sending calibrate command: {e}")

    def send_reset_yaw(self):
        """Send yaw reset command"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"z\n")
            except serial.SerialException as e:
                print(f"Error sending yaw reset: {e}")

    def save_to_eeprom(self):
        """Save current parameters to EEPROM"""
        if not self.ser or not self.ser.is_open:
            QMessageBox.warning(self, "Error", "Not connected to ESP32")
            return
            
        # First send current parameters
        self.send_params()
        
        # Then send save command after small delay
        QTimer.singleShot(100, lambda: self.ser.write(b"f\n") if self.ser else None)
        
        QMessageBox.information(self, "Success", 
                              "Parameters saved to ESP32's EEPROM")

    def request_current_params(self):
        """Request current parameters from ESP32"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"?\n")
            except serial.SerialException as e:
                print(f"Error requesting parameters: {e}")

    def check_serial_status(self):
        """Check if we've received any data from ESP32"""
        if self.ser and self.ser.is_open and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("params:"):
                    params = line[7:].split(',')
                    if len(params) == 3:
                        accel = float(params[0])
                        gyro = float(params[1])
                        comp = float(params[2])
                        
                        # Block signals to prevent feedback
                        self.accel_slider.blockSignals(True)
                        self.gyro_slider.blockSignals(True)
                        self.comp_slider.blockSignals(True)
                        self.accel_spinbox.blockSignals(True)
                        self.gyro_spinbox.blockSignals(True)
                        self.comp_spinbox.blockSignals(True)
                        
                        # Update controls
                        self.accel_slider.setValue(int(accel * 100))
                        self.gyro_slider.setValue(int(gyro * 100))
                        self.comp_slider.setValue(int(comp * 100))
                        self.accel_spinbox.setValue(accel)
                        self.gyro_spinbox.setValue(gyro)
                        self.comp_spinbox.setValue(comp)
                        
                        # Update parameters
                        self.params['accel_filter'] = accel
                        self.params['gyro_filter'] = gyro
                        self.params['comp_filter'] = comp
                        
                        # Restore signals
                        self.accel_slider.blockSignals(False)
                        self.gyro_slider.blockSignals(False)
                        self.comp_slider.blockSignals(False)
                        self.accel_spinbox.blockSignals(False)
                        self.gyro_spinbox.blockSignals(False)
                        self.comp_spinbox.blockSignals(False)
            except Exception as e:
                print(f"Serial read error: {e}")

    def safe_exit(self):
        """Safely close the GUI and notify ESP32 to switch to standalone mode"""
        # Notify ESP32 to switch to standalone mode
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"x\n")  # Custom command to ESP32
                time.sleep(0.1)         # Give ESP32 time to process
                sys.exit(0)         # Exit the program
            except:
                pass

        # Stop timers
        self.status_timer.stop()

        # Close serial
        if self.ser and self.ser.is_open:
            self.ser.close()

        # Close GUI
        self.close()
        print("L1")

    def closeEvent(self, event):
        """Handle window close (X button) the same way as safe_exit()"""
        self.safe_exit()
        event.accept()
        print("L2")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StabilizerConfigGUI()
    window.show()
    sys.exit(app.exec_())