# main.py
from cube_visualizer import CubeVisualizer
import time

# Start the visualizer in background
cube = CubeVisualizer()
cube.start()

# Main app loop
try:
    while True:
        print("Main app is running...")
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")
    cube.stop()
    cube.join()
