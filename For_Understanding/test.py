import matplotlib.pyplot as plt
import numpy as np

# Simulation parameters
dt = 0.02  # 50 Hz
time = np.arange(0, 0.4, dt)  # 0.4 seconds

# Gains
Kp = 1.0
Ki = 2.0
Kd = 0.5

# Initial conditions
manual_servo = 20  # percent
roll_angle = 20  # degrees (same as error because setpoint is 0)

# Simulating two cases: without and with bumpless transfer
output_no_bumpless = []
output_with_bumpless = []

# Without bumpless transfer
integral_nb = 0
prev_error_nb = 0

# With bumpless transfer (we calculate the required integral to match output)
error = -roll_angle
derivative = (-roll_angle - 0) / dt
target_output = manual_servo
integral_b = (target_output - (Kp * error) - (Kd * derivative)) / Ki
prev_error_b = error

# Let's simulate both for a few time steps
for t in time:
    # No bumpless
    error_nb = -roll_angle
    integral_nb += error_nb * dt
    derivative_nb = (error_nb - prev_error_nb) / dt
    output_nb = Kp * error_nb + Ki * integral_nb + Kd * derivative_nb
    output_no_bumpless.append(output_nb)
    prev_error_nb = error_nb

    # With bumpless
    error_b = -roll_angle
    integral_b += error_b * dt
    derivative_b = (error_b - prev_error_b) / dt
    output_b = Kp * error_b + Ki * integral_b + Kd * derivative_b
    output_with_bumpless.append(output_b)
    prev_error_b = error_b

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time, output_no_bumpless, label="Without Bumpless Transfer", linestyle='--', color='red')
plt.plot(time, output_with_bumpless, label="With Bumpless Transfer", linestyle='-', color='green')
plt.axhline(manual_servo, color='gray', linestyle=':', label='Manual Servo Position (20%)')
plt.title("PID Output Response on Mode Switch")
plt.xlabel("Time (s)")
plt.ylabel("PID Output (%)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
