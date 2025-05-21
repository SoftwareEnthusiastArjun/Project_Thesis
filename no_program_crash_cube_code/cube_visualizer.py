# cube_visualizer.py
import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import time

class CubeVisualizer(threading.Thread):
    def __init__(self, port='COM8', baudrate=38400):
        super().__init__()
        self.ser = self.init_serial(port, baudrate)
        self.ax = self.ay = self.az = 0.0
        self.yaw_mode = False
        self.running = True
        self.daemon = True  # Ends thread when main program exits

    def init_serial(self, port, baudrate):
        try:
            return serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            return None

    def init_gl(self):
        glViewport(0, 0, 640, 480)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 640 / 480, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def draw_cube(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -7.0)

        if self.yaw_mode:
            glRotatef(self.az, 0.0, 1.0, 0.0)
        glRotatef(self.ay, 1.0, 0.0, 0.0)
        glRotatef(-self.ax, 0.0, 0.0, 1.0)

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)  # Front
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)  # Back
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)  # Top
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)  # Bottom
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)  # Left
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)  # Right
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    def run(self):
        if not self.ser:
            return

        pygame.init()
        screen = pygame.display.set_mode((640, 480), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("MPU6050 3D Cube")
        clock = pygame.time.Clock()

        self.init_gl()

        while self.running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False  # Exit loop cleanly
                elif event.type == KEYDOWN:
                    if event.key == K_z:
                        self.yaw_mode = not self.yaw_mode
                        self.ser.write(b'z\n')

            try:
                self.ser.write(b'.\n')
                line = self.ser.readline().decode().strip()
                if line and (line[0].isdigit() or line[0] == '-' or line[0] == '0'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        self.ax, self.ay, self.az = [float(p) for p in parts]
            except Exception as e:
                print(f"Serial error: {e}")

            self.draw_cube()
            pygame.display.flip()
            clock.tick(60)

        # Cleanup
        pygame.quit()
        if self.ser:
            self.ser.close()

    def stop(self):
        """Call this to safely shut down the visualizer from outside."""
        pygame.display.quit()  # Only close window
        # self.running = False
        

