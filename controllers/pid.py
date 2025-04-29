
class PIDController:
    def __init__(self, kp, ki, kd, imax=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax
        self.integral = 0
        self.last_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        if self.imax is not None:
            self.integral = max(min(self.integral, self.imax), -self.imax)  # anti-windup
        derivative = (error - self.last_error) / dt
        self.last_error = error
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(u, 1.0), -1.0)
   
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
        pwm_output = np.clip(pwm_output, -1.0, 1.0)  # Normalized PWM

        return pwm_output
