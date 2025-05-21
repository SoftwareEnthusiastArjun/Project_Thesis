import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import sys
import time

# Global variables for orientation angles
ax = ay = az = 0.0  # Angles of rotation around x, y, z axes
yaw_mode = False  # Flag to toggle yaw rotation mode

# Function to initialize serial communication
def init_serial():
    """
    Initializes serial communication with the MPU6050 sensor.
    Sets the port to COM8 and baud rate to 38400.
    Returns a serial object if successful, else prints an error message and returns None.
    """
    try:
        return serial.Serial('COM8', 38400, timeout=1)
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        return None

# Function to initialize OpenGL settings
def init_gl():
    """
    Initializes the OpenGL environment with perspective settings.
    Sets up the view, projection, and camera properties.
    Configures OpenGL to render in smooth shading and with depth testing enabled.
    """
    glViewport(0, 0, 640, 480)  # Set the viewport size
    glMatrixMode(GL_PROJECTION)  # Set matrix mode to projection
    glLoadIdentity()  # Load identity matrix for projection
    gluPerspective(45, 640 / 480, 0.1, 100.0)  # Set the perspective projection
    glMatrixMode(GL_MODELVIEW)  # Switch back to modelview matrix mode
    glLoadIdentity()  # Load identity matrix for modelview
    glShadeModel(GL_SMOOTH)  # Use smooth shading for rendering
    glClearColor(0.0, 0.0, 0.0, 0.0)  # Set background color to black
    glClearDepth(1.0)  # Set depth buffer to 1.0
    glEnable(GL_DEPTH_TEST)  # Enable depth testing for 3D rendering
    glDepthFunc(GL_LEQUAL)  # Set depth function for rendering
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)  # Set perspective correction for smooth rendering

# Function to draw a rotating cube based on orientation angles
def draw_cube(ax, ay, az, yaw_mode):
    """
    Draws a 3D cube with different colored faces.
    Rotates the cube based on the input angles (ax, ay, az).
    If yaw_mode is enabled, only yaw rotation (around the z-axis) is applied.
    """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # Clear the color and depth buffers
    glLoadIdentity()  # Reset the modelview matrix
    glTranslatef(0.0, 0.0, -7.0)  # Move the cube along the z-axis to make it visible

    if yaw_mode:
        glRotatef(az, 0.0, 1.0, 0.0)  # Apply yaw rotation (around z-axis)
    glRotatef(ay, 1.0, 0.0, 0.0)  # Apply pitch rotation (around x-axis)
    glRotatef(-ax, 0.0, 0.0, 1.0)  # Apply roll rotation (around y-axis)

    # Draw the cube faces with different colors
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

# Main function to run the program
def main():
    """
    Main function that sets up serial communication, initializes OpenGL,
    and runs the main loop for handling events and rendering the cube.
    """
    global ax, ay, az, yaw_mode

    # Initialize serial communication
    ser = init_serial()
    if not ser:
        return

    # Initialize pygame and OpenGL
    pygame.init()
    screen = pygame.display.set_mode((640, 480), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("MPU6050 3D Cube")  # Set window title
    clock = pygame.time.Clock()

    init_gl()  # Initialize OpenGL settings

    # Main loop
    while True:
        for event in pygame.event.get():
            # Handle window close event
            if event.type == QUIT:
                pygame.quit()
                ser.close()
                sys.exit()
            # Handle key press to toggle yaw mode
            elif event.type == KEYDOWN:
                if event.key == K_z:
                    yaw_mode = not yaw_mode
                    ser.write(b'z\n')  # Send command to zero yaw angle

        # Request sensor data for angles (ax, ay, az)
        try:
            ser.write(b'.\n')  # Send command to request data
            line = ser.readline().decode().strip()  # Read the response from serial
            if line and (line[0].isdigit() or line[0] == '-' or line[0] == '0'):
                parts = line.split(',')
                if len(parts) == 3:
                    ax, ay, az = [float(p) for p in parts]  # Parse and assign angles
        except Exception as e:
            print(f"Serial error: {e}")  # Print any serial errors

        # Draw the cube with updated angles
        draw_cube(ax, ay, az, yaw_mode)
        pygame.display.flip()  # Update the display
        clock.tick(60)  # Control the frame rate

# Entry point of the program
if __name__ == '__main__':
    main()
