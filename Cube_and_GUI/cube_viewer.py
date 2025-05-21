import serial
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import threading

import socket
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

class CubeViewer:
    def __init__(self, host='esp32.local', port=12345):
        self.host = host
        self.port = port
        self.running = False
        self.sock = None

    def init_gl(self):
        glViewport(0, 0, 640, 480)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 640 / 480, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)

    def draw_cube(self, gx, gy):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -7.0)
        glRotatef(gy, 1.0, 0.0, 0.0)  # Pitch
        glRotatef(-gx, 0.0, 0.0, 1.0) # Roll

        glBegin(GL_QUADS)
        glColor3f(0, 1, 0)
        glVertex3f(1, 1, -1)
        glVertex3f(-1, 1, -1)
        glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1)

        glColor3f(1, 0.5, 0)
        glVertex3f(1, -1, 1)
        glVertex3f(-1, -1, 1)
        glVertex3f(-1, -1, -1)
        glVertex3f(1, -1, -1)

        glColor3f(1, 0, 0)
        glVertex3f(1, 1, 1)
        glVertex3f(-1, 1, 1)
        glVertex3f(-1, -1, 1)
        glVertex3f(1, -1, 1)

        glColor3f(1, 1, 0)
        glVertex3f(1, -1, -1)
        glVertex3f(-1, -1, -1)
        glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1)

        glColor3f(0, 0, 1)
        glVertex3f(-1, 1, 1)
        glVertex3f(-1, 1, -1)
        glVertex3f(-1, -1, -1)
        glVertex3f(-1, -1, 1)

        glColor3f(1, 0, 1)
        glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1)
        glVertex3f(1, -1, 1)
        glVertex3f(1, -1, -1)
        glEnd()

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3)
            self.sock.connect((self.host, self.port))
            self.sock.sendall(b"startCubeStream\n")
        except Exception as e:
            print(f"Socket error: {e}")
            return

        pygame.init()
        screen = pygame.display.set_mode((640, 480), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Cube Visualizer")
        self.init_gl()

        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False

            try:
                data = self.sock.recv(64).decode().strip()
                lines = data.split('\n')
                gx, gy = 0.0, 0.0
                for line in lines:
                    if line:
                        parts = list(map(float, line.strip().split(',')))
                        if len(parts) >= 2:
                            gx, gy = parts[0], parts[1]
                            break
            except:
                gx, gy = 0.0, 0.0

            self.draw_cube(gx, gy)
            pygame.display.flip()
            pygame.time.wait(16)

        try:
            self.sock.sendall(b"stopCubeStream\n")
            self.sock.close()
        except:
            pass
        pygame.quit()

    def stop(self):
        self.running = False


# Optional: for standalone testing
if __name__ == '__main__':
    viewer = CubeViewer()
    viewer.run()
