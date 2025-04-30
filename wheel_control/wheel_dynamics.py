class WheelDynamics:
    def __init__(self, tau=0.2, K=1.0):
        self.tau = tau  # time constant
        self.K = K      # static gain
        self.omega = 0.0  # current angular speed

    def update(self, pwm_input, dt):
        # First-order dynamic model: tau * dω/dt + ω = K * u
        omega_dot = (self.K * pwm_input - self.omega) / self.tau
        self.omega += omega_dot * dt
        return self.omega