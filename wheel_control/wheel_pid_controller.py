import numpy as np

class WheelPIDController:
    def __init__(self, kp, ki, kd, imax=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax
        self.integral = 0
        self.last_error = 0

    def update(self, omega_ref, omega_measured, dt):
        error = omega_ref - omega_measured

        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.imax, self.imax)

        derivative = (error - self.last_error) / dt
        self.last_error = error

        pwm_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        pwm_output = np.clip(pwm_output, -1.0, 1.0)

        return pwm_output
