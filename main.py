from controllers.pid import PIDController
from models.simple_robot import RobotModel
import matplotlib.pyplot as plt
import numpy as np

# Time settings
dt = 0.01
T = 10
steps = int(T / dt)
time = np.arange(0, T, dt)

# Reference velocity
v_ref = np.ones(steps) * 0.5

# Initialization
robot = RobotModel()
controller = PIDController(kp=2.0, ki=1.0, kd=0.1)

v = np.zeros(steps)
v_measured = np.zeros(steps)
u = np.zeros(steps)
error = np.zeros(steps)

for t in range(1, steps):
    error[t] = v_ref[t] - v_measured[t-1]
    u[t] = controller.update(error[t], dt)
    v[t] = robot.update(u[t])
    v_measured[t] = v[t] + np.random.normal(0, 0.02)  # sensor noise

# Plotting
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time, v_ref, '--', label='v_ref')
plt.plot(time, v, label='v_real')
plt.plot(time, v_measured, label='v_measured')
plt.ylabel('Velocity (m/s)')
plt.title('System Response')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, error, label='Error', color='red')
plt.ylabel('Error')
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, u, label='Control (u)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Command')
plt.grid()

plt.tight_layout()
plt.show()
