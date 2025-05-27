Now the Cube_and_GUI.ino and GUI.py can do this:
1. Connect to wifi(newly modified for controlling servo: but now it can still finish set up even when wifi is not there(it waits for 10 sec, then skip wifi connecting 
part and move on with other fuc))
2. Read, edit and save filter values.
3. Based on PWM waves, see % of i/p of 3 channels from rc receiver
4. See if autopilot is on/off based on pwm%
5. read IMU sensor's filterd values 
6. Send then to cube visualizer to see graphics
7. Control servo(Pitch and roll) even when the wifi is connected or disconnected
--------------------------------
wiring:
SDA:D21
SCL:D22
VCC(MPU):3V3
gnd

ail(ch1): d15
ele(ch2):rx2
rud(ch4):tx2
auto(ch5):d18
pwr(vin),gnd