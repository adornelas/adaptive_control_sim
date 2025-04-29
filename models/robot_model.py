class RobotModel:
    def __init__(self, R=0.015, L=0.075, tau=0.2, v_max=113.6):
        self.R = R          # wheel radius
        self.L = L          # distance between wheels
        self.tau = tau      # time constant of motor
        self.v_max = v_max  # rad/s
        self.v_r = 0.0      # right wheel speed
        self.v_l = 0.0      # left wheel speed
        self.v = 0.0        # linear velocity
        self.w = 0.0        # angular velocity

    def update(self, pwm_r, pwm_l, dt):
        # PWM [-1, 1] assumed to map directly to desired speed
        v_r_cmd = pwm_r * self.v_max
        v_l_cmd = pwm_l * self.v_max

        # First-order response of each motor
        self.v_r += (v_r_cmd - self.v_r) * dt / self.tau
        self.v_l += (v_l_cmd - self.v_l) * dt / self.tau

        # Differential drive kinematics
        self.v = self.R / 2 * (self.v_r + self.v_l)
        self.w = self.R / self.L * (self.v_r - self.v_l)