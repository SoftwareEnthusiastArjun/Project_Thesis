import sys
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QSlider, QPushButton, 
                            QDoubleSpinBox, QGroupBox, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
"""Reads current FW congig and shows it in a GUI.
Allows to change the parameters(acc, gyro, filter coeff) and send them to the ESP32.
Reads the current angles from the ESP32 and use it to contol the 3D cube. """
class StabilizerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Serial connection
        self.ser = None
        self.init_serial()
        
        # Initialize pygame for visualization
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480), OPENGL | DOUBLEBUF)
        pygame.display.set_caption("MPU6050 Stabilizer Visualization")
        self.init_gl()
        
        # Current angles
        self.ax = self.ay = self.az = 0.0
        self.yaw_mode = False
        
        # Filter parameters (initial values)
        self.params = {
            'accel_filter': 0.3,
            'gyro_filter': 0.08,
            'comp_filter': 0.7,
            'sample_rate': 50.0
        }
        
        # Setup UI
        self.init_ui()
        
        # Timer for data updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(20)  # ~50Hz
        
        # Timer for visualization
        self.viz_timer = QTimer(self)
        self.viz_timer.timeout.connect(self.update_visualization)
        self.viz_timer.start(16)  # ~60Hz
        
        # Request current parameters from ESP32 after connection is established
        QTimer.singleShot(1000, self.request_current_parameters)
    
    def init_serial(self):
        """Initialize serial connection"""
        try:
            self.ser = serial.Serial('COM8', 38400, timeout=1)
        except serial.SerialException as e:
            QMessageBox.critical(self, "Serial Error", f"Failed to open serial port: {str(e)}")
            self.ser = None
    
    def request_current_parameters(self):
        """Request current parameters from ESP32"""
        if self.ser:
            self.ser.write(b"?\n")  # This requests the current parameters from ESP32
    
    def init_ui(self):
        # Main widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Control panel
        control_panel = QGroupBox("Filter Controls")
        control_layout = QVBoxLayout()
        
        # Accel Filter Control
        accel_group = QGroupBox("Accelerometer Filter (0.01-1.0)")
        accel_layout = QVBoxLayout()
        self.accel_slider = QSlider(Qt.Horizontal)
        self.accel_slider.setRange(1, 100)
        self.accel_slider.setValue(int(self.params['accel_filter'] * 100))
        self.accel_slider.valueChanged.connect(self.update_accel_filter)
        self.accel_spinbox = QDoubleSpinBox()
        self.accel_spinbox.setRange(0.01, 1.0)
        self.accel_spinbox.setSingleStep(0.01)
        self.accel_spinbox.setValue(self.params['accel_filter'])
        self.accel_spinbox.valueChanged.connect(self.update_accel_filter)
        accel_layout.addWidget(self.accel_slider)
        accel_layout.addWidget(self.accel_spinbox)
        accel_group.setLayout(accel_layout)
        
        # Gyro Filter Control
        gyro_group = QGroupBox("Gyroscope Filter (0.01-1.0)")
        gyro_layout = QVBoxLayout()
        self.gyro_slider = QSlider(Qt.Horizontal)
        self.gyro_slider.setRange(1, 100)
        self.gyro_slider.setValue(int(self.params['gyro_filter'] * 100))
        self.gyro_slider.valueChanged.connect(self.update_gyro_filter)
        self.gyro_spinbox = QDoubleSpinBox()
        self.gyro_spinbox.setRange(0.01, 1.0)
        self.gyro_spinbox.setSingleStep(0.01)
        self.gyro_spinbox.setValue(self.params['gyro_filter'])
        self.gyro_spinbox.valueChanged.connect(self.update_gyro_filter)
        gyro_layout.addWidget(self.gyro_slider)
        gyro_layout.addWidget(self.gyro_spinbox)
        gyro_group.setLayout(gyro_layout)
        
        # Complementary Filter Control
        comp_group = QGroupBox("Complementary Filter (0.01-1.0)")
        comp_layout = QVBoxLayout()
        self.comp_slider = QSlider(Qt.Horizontal)
        self.comp_slider.setRange(1, 100)
        self.comp_slider.setValue(int(self.params['comp_filter'] * 100))
        self.comp_slider.valueChanged.connect(self.update_comp_filter)
        self.comp_spinbox = QDoubleSpinBox()
        self.comp_spinbox.setRange(0.01, 1.0)
        self.comp_spinbox.setSingleStep(0.01)
        self.comp_spinbox.setValue(self.params['comp_filter'])
        self.comp_spinbox.valueChanged.connect(self.update_comp_filter)
        comp_layout.addWidget(self.comp_slider)
        comp_layout.addWidget(self.comp_spinbox)
        comp_group.setLayout(comp_layout)
        
        # Action buttons
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout()
        
        self.calibrate_btn = QPushButton("Recalibrate Gyro")
        self.calibrate_btn.clicked.connect(self.send_calibrate)
        
        self.yaw_btn = QPushButton("Toggle Yaw Mode")
        self.yaw_btn.clicked.connect(self.toggle_yaw_mode)
        
        self.flash_btn = QPushButton("Save to ESP32 (Permanent)")
        self.flash_btn.clicked.connect(self.flash_values)
        
        action_layout.addWidget(self.calibrate_btn)
        action_layout.addWidget(self.yaw_btn)
        action_layout.addWidget(self.flash_btn)
        action_group.setLayout(action_layout)
        
        # Add all controls to main layout
        control_layout.addWidget(accel_group)
        control_layout.addWidget(gyro_group)
        control_layout.addWidget(comp_group)
        control_layout.addWidget(action_group)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)

        # Add control panel and visualization to main window
        main_layout.addWidget(control_panel, stretch=1)
        
        # Window settings
        self.setWindowTitle("MPU6050 Stabilizer Tuner")
        self.resize(1000, 600)
        
    def update_accel_filter(self, value):
        if isinstance(value, int):
            value = value / 100.0
        self.params['accel_filter'] = value
        # Update both controls
        self.accel_spinbox.setValue(value)
        if isinstance(value, float):
            self.accel_slider.setValue(int(value * 100))
        self.send_params()
        
    def update_gyro_filter(self, value):
        if isinstance(value, int):
            value = value / 100.0
        self.params['gyro_filter'] = value
        self.gyro_spinbox.setValue(value)
        if isinstance(value, float):
            self.gyro_slider.setValue(int(value * 100))
        self.send_params()
        
    def update_comp_filter(self, value):
        if isinstance(value, int):
            value = value / 100.0
        self.params['comp_filter'] = value
        self.comp_spinbox.setValue(value)
        if isinstance(value, float):
            self.comp_slider.setValue(int(value * 100))
        self.send_params()
        
    def send_params(self):
        """Send current parameters to ESP32"""
        if self.ser:
            param_str = f"p{self.params['accel_filter']:.4f},{self.params['gyro_filter']:.4f},{self.params['comp_filter']:.4f}\n"
            self.ser.write(param_str.encode())
        
    def send_calibrate(self):
        """Send calibration command"""
        if self.ser:
            self.ser.write(b"c\n")
        
    def toggle_yaw_mode(self):
        """Toggle yaw mode and reset yaw angle"""
        self.yaw_mode = not self.yaw_mode
        if self.ser:
            self.ser.write(b"z\n")
    
    def flash_values(self):
        """Save current values permanently to ESP32"""
        if not self.ser:
            QMessageBox.warning(self, "Error", "Not connected to ESP32")
            return
            
        # First send current parameters
        self.send_params()
        
        # Then send flash command after small delay
        QTimer.singleShot(100, lambda: self.ser.write(b"f\n") if self.ser else None)
        
        QMessageBox.information(self, "Success", 
                              "Parameters saved to ESP32's EEPROM.\n"
                              "They will persist after reset.")
        
    def update_data(self):
        """Read orientation data from the ESP32 via serial."""
        if not self.ser:
            return

        try:
            # Request new angle data
            self.ser.write(b".\n")  # Ask for pitch, roll, yaw
            line = self.ser.readline().decode().strip()

            if line.startswith("params:"):
                # Received filter parameters
                try:
                    params = line[7:].split(',')
                    if len(params) == 3:
                        accel = float(params[0])
                        gyro = float(params[1])
                        comp = float(params[2])

                        # Block signals temporarily to prevent feedback
                        self.accel_slider.blockSignals(True)
                        self.gyro_slider.blockSignals(True)
                        self.comp_slider.blockSignals(True)
                        self.accel_spinbox.blockSignals(True)
                        self.gyro_spinbox.blockSignals(True)
                        self.comp_spinbox.blockSignals(True)

                        self.accel_slider.setValue(int(accel * 100))
                        self.gyro_slider.setValue(int(gyro * 100))
                        self.comp_slider.setValue(int(comp * 100))
                        self.accel_spinbox.setValue(accel)
                        self.gyro_spinbox.setValue(gyro)
                        self.comp_spinbox.setValue(comp)

                        self.accel_slider.blockSignals(False)
                        self.gyro_slider.blockSignals(False)
                        self.comp_slider.blockSignals(False)
                        self.accel_spinbox.blockSignals(False)
                        self.gyro_spinbox.blockSignals(False)
                        self.comp_spinbox.blockSignals(False)

                        # Update local parameter cache
                        self.params['accel_filter'] = accel
                        self.params['gyro_filter'] = gyro
                        self.params['comp_filter'] = comp
                except ValueError:
                    pass

            elif line and (line[0].isdigit() or line[0] == '-' or line[0] == '0'):
                # Received angle data
                try:
                    angles = [float(x) for x in line.split(',')]
                    if len(angles) == 3:
                        self.ax, self.ay, self.az = angles
                except ValueError:
                    pass

        except Exception as e:
            print("Serial read error:", e)

                
    def init_gl(self):
        glViewport(0, 0, 640, 480)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 640/480, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        
    def draw_text(self, position, text_string):     
        font = pygame.font.SysFont("Courier", 18, True)
        text_surface = font.render(text_string, True, (255,255,255,255), (0,0,0,255))     
        text_data = pygame.image.tostring(text_surface, "RGBA", True)     
        glRasterPos3d(*position)     
        glDrawPixels(text_surface.get_width(), text_surface.get_height(), 
                    GL_RGBA, GL_UNSIGNED_BYTE, text_data)
        
    def draw_cube(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)
        
        # Display current parameters
        param_text = (f"Accel: {self.params['accel_filter']:.2f} | "
                     f"Gyro: {self.params['gyro_filter']:.2f} | "
                     f"Comp: {self.params['comp_filter']:.2f}")
        self.draw_text((-2, -2, 2), param_text)
        
        # Apply rotations
        if self.yaw_mode:
            glRotatef(self.az, 0.0, 1.0, 0.0)  # Yaw
        glRotatef(self.ay, 1.0, 0.0, 0.0)      # Pitch
        glRotatef(-self.ax, 0.0, 0.0, 1.0)     # Roll
        
        # Draw cube
        glBegin(GL_QUADS)
        # Front face
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)
        
        # Back face
        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)
        
        # Top face
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        
        # Bottom face
        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)
        
        # Left face
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        
        # Right face
        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()
        
    def update_visualization(self):
        self.draw_cube()
        pygame.display.flip()
        
        # Process pygame events to keep window responsive
        for event in pygame.event.get():
            if event.type == QUIT:
                self.close()
                
    def closeEvent(self, event):
        self.timer.stop()
        self.viz_timer.stop()
        if self.ser:
            self.ser.close()
        pygame.quit()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = StabilizerGUI()
    gui.show()
    sys.exit(app.exec_())