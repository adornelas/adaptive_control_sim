import numpy as np

class FirmwareController:
    def __init__(self, R=0.015, L=0.075, kp=10.0, ki=1.1, kd = 0.05, imax=64.0, v_pwm_max=1.0):
        self.R = R
        self.L = L
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax
        self.v_pwm_max = v_pwm_max

        self.w_integral = 0
        self.last_w_error = 0

        self.v_r = 0.0
        self.v_l = 0.0

        self.v = 0.0
        self.w = 0.0

    # receives v_ref, w_ref, w_measured (and pointer to global error?)
    def update(self, v_ref, w_error, dt):

        w_cmd = self.PID(w_error, dt)

        # v_ref é enviado diretamente
        v_cmd = v_ref

        #speed2motors
        v_r_cmd = (v_cmd + (self.L / 2) * w_cmd) / self.R
        v_l_cmd = (v_cmd - (self.L / 2) * w_cmd) / self.R

        #angvel2pwm
        v_r_cmd = v_r_cmd * self.v_pwm_max / (self.v_pwm_max / self.R) 
        v_l_cmd = v_l_cmd * self.v_pwm_max / (self.v_pwm_max / self.R)

        # saturation
        v_r_cmd = np.clip(v_r_cmd, -self.v_pwm_max, self.v_pwm_max)
        v_l_cmd = np.clip(v_l_cmd, -self.v_pwm_max, self.v_pwm_max)

        return v_r_cmd, v_l_cmd
       
    def PID(self, w_error, dt):
        self.w_integral += w_error * dt
        self.w_integral = np.clip(self.w_integral, -self.imax, self.imax)   # anti-windup

        P = w_error * self.kp
        I = self.w_integral * self.ki
        D = (w_error - self.last_w_error) * self.kd

        self.last_w_error = w_error

        u = P + I + D

        return u
