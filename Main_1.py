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

"""
MPU6050 Stabilizer GUI Application

This application:
1. Reads current firmware configuration from ESP32
2. Provides GUI controls to adjust filter parameters (accelerometer, gyro, complementary filter)
3. Sends updated parameters to ESP32
4. Visualizes the 3D orientation of the MPU6050 sensor in real-time
5. Allows saving parameters to ESP32's EEPROM
"""

class StabilizerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Serial connection setup
        self.ser = None  # Will hold the serial connection
        self.init_serial()  # Initialize serial connection
        
        # Initialize pygame for 3D visualization
        pygame.init()
        # Set up OpenGL display with 640x480 resolution
        self.screen = pygame.display.set_mode((640, 480), OPENGL | DOUBLEBUF)
        pygame.display.set_caption("MPU6050 Stabilizer Visualization")
        self.init_gl()  # Initialize OpenGL settings
        
        # Current orientation angles (pitch, roll, yaw)
        self.ax = self.ay = self.az = 0.0
        self.yaw_mode = False  # Toggle for yaw visualization
        
        # Default filter parameters
        self.params = {
            'accel_filter': 0.3,    # Accelerometer filter coefficient
            'gyro_filter': 0.08,    # Gyroscope filter coefficient
            'comp_filter': 0.7,      # Complementary filter coefficient
            'sample_rate': 50.0      # Sample rate in Hz
        }
        
        # Initialize the user interface
        self.init_ui()
        
        # Setup timers for periodic updates
        self.timer = QTimer(self)  # For data updates
        self.timer.timeout.connect(self.update_data)
        self.timer.start(20)  # ~50Hz update rate
        
        self.viz_timer = QTimer(self)  # For visualization updates
        self.viz_timer.timeout.connect(self.update_visualization)
        self.viz_timer.start(16)  # ~60Hz refresh rate
        
        # Request current parameters after 1 second delay (allows connection to establish)
        QTimer.singleShot(1000, self.request_current_parameters)
    
    def init_serial(self):
        """Initialize serial connection to ESP32"""
        try:
            # Attempt to open serial port COM8 at 38400 baud
            self.ser = serial.Serial('COM8', 38400, timeout=1)
        except serial.SerialException as e:
            # Show error message if connection fails
            QMessageBox.critical(self, "Serial Error", f"Failed to open serial port: {str(e)}")
            self.ser = None
    
    def request_current_parameters(self):
        """Request current filter parameters from ESP32"""
        if self.ser:
            self.ser.write(b"?\n")  # Send query command
    
    def init_ui(self):
        """Initialize the main user interface"""
        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Control panel setup
        control_panel = QGroupBox("Filter Controls")
        control_layout = QVBoxLayout()
        
        # Accelerometer Filter Control Group
        accel_group = QGroupBox("Accelerometer Filter (0.01-1.0)")
        accel_layout = QVBoxLayout()
        # Slider for accelerometer filter
        self.accel_slider = QSlider(Qt.Horizontal)
        self.accel_slider.setRange(1, 100)  # Maps to 0.01-1.0
        self.accel_slider.setValue(int(self.params['accel_filter'] * 100))
        self.accel_slider.valueChanged.connect(self.update_accel_filter)
        # Spinbox for precise value entry
        self.accel_spinbox = QDoubleSpinBox()
        self.accel_spinbox.setRange(0.01, 1.0)
        self.accel_spinbox.setSingleStep(0.01)
        self.accel_spinbox.setValue(self.params['accel_filter'])
        self.accel_spinbox.valueChanged.connect(self.update_accel_filter)
        accel_layout.addWidget(self.accel_slider)
        accel_layout.addWidget(self.accel_spinbox)
        accel_group.setLayout(accel_layout)
        
        # Gyroscope Filter Control Group (similar structure)
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
        
        # Complementary Filter Control Group (similar structure)
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
        
        # Action Buttons Group
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout()
        
        # Calibration button
        self.calibrate_btn = QPushButton("Recalibrate Gyro")
        self.calibrate_btn.clicked.connect(self.send_calibrate)
        
        # Yaw mode toggle button
        self.yaw_btn = QPushButton("Toggle Yaw Mode")
        self.yaw_btn.clicked.connect(self.toggle_yaw_mode)
        
        # Save to EEPROM button
        self.flash_btn = QPushButton("Save to ESP32 (Permanent)")
        self.flash_btn.clicked.connect(self.flash_values)
        
        action_layout.addWidget(self.calibrate_btn)
        action_layout.addWidget(self.yaw_btn)
        action_layout.addWidget(self.flash_btn)
        action_group.setLayout(action_layout)
        
        # Add all control groups to main control layout
        control_layout.addWidget(accel_group)
        control_layout.addWidget(gyro_group)
        control_layout.addWidget(comp_group)
        control_layout.addWidget(action_group)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)

        # Add control panel to main window
        main_layout.addWidget(control_panel, stretch=1)
        
        # Window settings
        self.setWindowTitle("MPU6050 Stabilizer Tuner")
        self.resize(1000, 600)
        
    def update_accel_filter(self, value):
        """Update accelerometer filter value from UI control"""
        if isinstance(value, int):
            value = value / 100.0  # Convert slider integer to float
        self.params['accel_filter'] = value
        # Synchronize both controls
        self.accel_spinbox.setValue(value)
        if isinstance(value, float):
            self.accel_slider.setValue(int(value * 100))
        self.send_params()  # Send updated value to ESP32
        
    def update_gyro_filter(self, value):
        """Update gyroscope filter value from UI control"""
        if isinstance(value, int):
            value = value / 100.0
        self.params['gyro_filter'] = value
        self.gyro_spinbox.setValue(value)
        if isinstance(value, float):
            self.gyro_slider.setValue(int(value * 100))
        self.send_params()
        
    def update_comp_filter(self, value):
        """Update complementary filter value from UI control"""
        if isinstance(value, int):
            value = value / 100.0
        self.params['comp_filter'] = value
        self.comp_spinbox.setValue(value)
        if isinstance(value, float):
            self.comp_slider.setValue(int(value * 100))
        self.send_params()
        
    def send_params(self):
        """Send current parameters to ESP32 in format 'p0.3000,0.0800,0.7000'"""
        if self.ser:
            param_str = f"p{self.params['accel_filter']:.4f},{self.params['gyro_filter']:.4f},{self.params['comp_filter']:.4f}\n"
            self.ser.write(param_str.encode())
        
    def send_calibrate(self):
        """Send gyroscope calibration command to ESP32"""
        if self.ser:
            self.ser.write(b"c\n")  # Calibration command
        
    def toggle_yaw_mode(self):
        """Toggle yaw visualization mode and reset yaw angle"""
        self.yaw_mode = not self.yaw_mode
        if self.ser:
            self.ser.write(b"z\n")  # Zero yaw command
    
    def flash_values(self):
        """Save current parameters to ESP32's EEPROM"""
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
        """Read and process data from ESP32 via serial"""
        if not self.ser:
            return

        try:
            # Request new angle data
            self.ser.write(b".\n")  # Data request command
            line = self.ser.readline().decode().strip()

            if line.startswith("params:"):
                # Received parameter update from ESP32
                try:
                    params = line[7:].split(',')
                    if len(params) == 3:
                        accel = float(params[0])
                        gyro = float(params[1])
                        comp = float(params[2])

                        # Block signals to prevent recursive updates
                        self.accel_slider.blockSignals(True)
                        self.gyro_slider.blockSignals(True)
                        self.comp_slider.blockSignals(True)
                        self.accel_spinbox.blockSignals(True)
                        self.gyro_spinbox.blockSignals(True)
                        self.comp_spinbox.blockSignals(True)

                        # Update UI controls
                        self.accel_slider.setValue(int(accel * 100))
                        self.gyro_slider.setValue(int(gyro * 100))
                        self.comp_slider.setValue(int(comp * 100))
                        self.accel_spinbox.setValue(accel)
                        self.gyro_spinbox.setValue(gyro)
                        self.comp_spinbox.setValue(comp)

                        # Re-enable signals
                        self.accel_slider.blockSignals(False)
                        self.gyro_slider.blockSignals(False)
                        self.comp_slider.blockSignals(False)
                        self.accel_spinbox.blockSignals(False)
                        self.gyro_spinbox.blockSignals(False)
                        self.comp_spinbox.blockSignals(False)

                        # Update local parameters
                        self.params['accel_filter'] = accel
                        self.params['gyro_filter'] = gyro
                        self.params['comp_filter'] = comp
                except ValueError:
                    pass

            elif line and (line[0].isdigit() or line[0] == '-' or line[0] == '0'):
                # Received angle data (pitch, roll, yaw)
                try:
                    angles = [float(x) for x in line.split(',')]
                    if len(angles) == 3:
                        self.ax, self.ay, self.az = angles
                except ValueError:
                    pass

        except Exception as e:
            print("Serial read error:", e)

    def init_gl(self):
        """Initialize OpenGL settings for 3D visualization"""
        glViewport(0, 0, 640, 480)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 640/480, 0.1, 100.0)  # Set perspective projection
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glShadeModel(GL_SMOOTH)  # Smooth shading
        glClearColor(0.0, 0.0, 0.0, 0.0)  # Black background
        glClearDepth(1.0)  # Depth buffer setup
        glEnable(GL_DEPTH_TEST)  # Enable depth testing
        glDepthFunc(GL_LEQUAL)  # Depth testing function
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)  # Best quality rendering
        
    def draw_text(self, position, text_string):     
        """Render text in the 3D scene"""
        font = pygame.font.SysFont("Courier", 18, True)
        text_surface = font.render(text_string, True, (255,255,255,255), (0,0,0,255))     
        text_data = pygame.image.tostring(text_surface, "RGBA", True)     
        glRasterPos3d(*position)     
        glDrawPixels(text_surface.get_width(), text_surface.get_height(), 
                    GL_RGBA, GL_UNSIGNED_BYTE, text_data)
        
    def draw_cube(self):
        """Draw the 3D cube representing MPU6050 orientation"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)  # Move camera back
        
        # Display current parameters as text overlay
        param_text = (f"Accel: {self.params['accel_filter']:.2f} | "
                     f"Gyro: {self.params['gyro_filter']:.2f} | "
                     f"Comp: {self.params['comp_filter']:.2f}")
        self.draw_text((-2, -2, 2), param_text)
        
        # Apply rotations based on current angles
        if self.yaw_mode:
            glRotatef(self.az, 0.0, 1.0, 0.0)  # Yaw rotation
        glRotatef(self.ay, 1.0, 0.0, 0.0)      # Pitch rotation
        glRotatef(-self.ax, 0.0, 0.0, 1.0)     # Roll rotation
        
        # Draw cube faces
        glBegin(GL_QUADS)
        # Front face (green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)
        
        # Back face (orange)
        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)
        
        # Top face (red)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        
        # Bottom face (yellow)
        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)
        
        # Left face (blue)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        
        # Right face (purple)
        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()
        
    def update_visualization(self):
        """Update the 3D visualization"""
        self.draw_cube()
        pygame.display.flip()  # Update the display
        
        # Process pygame events to keep window responsive
        for event in pygame.event.get():
            if event.type == QUIT:
                self.close()
                
    def closeEvent(self, event):
        """Cleanup when window is closed"""
        self.timer.stop()
        self.viz_timer.stop()
        if self.ser:
            self.ser.close()
        pygame.quit()
        event.accept()

if __name__ == '__main__':
    # Create and run the application
    app = QApplication(sys.argv)
    gui = StabilizerGUI()
    gui.show()
    sys.exit(app.exec_())