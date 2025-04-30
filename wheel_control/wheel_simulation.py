import numpy as np
import matplotlib.pyplot as plt
from wheel_pid_controller import WheelPIDController
from wheel_dynamics import WheelDynamics

# Simulation parameters
T = 5  # total time (s)
dt = 0.01  # time step (s)
steps = int(T / dt)
time = np.arange(0, T, dt)

# References
omega_r_ref = 10.0 * np.ones(steps)  # Right wheel desired angular velocity (rad/s)
omega_l_ref = 8.0 * np.ones(steps)   # Left wheel desired angular velocity (rad/s)

# Controllers
pid_r = WheelPIDController(kp=0.8, ki=0.2, kd=0.01)
pid_l = WheelPIDController(kp=0.8, ki=0.2, kd=0.01)

# Wheel dynamics
wheel_r = WheelDynamics(tau=0.2, K=10.0)
wheel_l = WheelDynamics(tau=0.2, K=10.0)

# Data recording
omega_r_measured = np.zeros(steps)
omega_l_measured = np.zeros(steps)
pwm_r = np.zeros(steps)
pwm_l = np.zeros(steps)

# Simulation loop
for t in range(1, steps):
    pwm_r[t] = pid_r.update(omega_r_ref[t], omega_r_measured[t-1], dt)
    pwm_l[t] = pid_l.update(omega_l_ref[t], omega_l_measured[t-1], dt)

    omega_r_measured[t] = wheel_r.update(pwm_r[t], dt)
    omega_l_measured[t] = wheel_l.update(pwm_l[t], dt)

# Plot results
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, omega_r_ref, '--', label='ω_r_ref')
plt.plot(time, omega_r_measured, label='ω_r_measured')
# plt.plot(time, omega_l_ref, '--', label='ω_l_ref')
# plt.plot(time, omega_l_measured, label='ω_l_measured')
plt.ylabel('Wheel Angular Speed (rad/s)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, omega_r_ref - omega_r_measured, label='Error ω_r')
# plt.plot(time, omega_l_ref - omega_l_measured, label='Error ω_l')
plt.ylabel('Speed Error (rad/s)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, pwm_r, label='PWM Right')
# plt.plot(time, pwm_l, label='PWM Left')
plt.ylabel('Control Signal (PWM)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
