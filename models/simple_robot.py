class RobotModel:
    def __init__(self):
        self.v = 0.0  # current linear velocity

    def update(self, u):
        # First-order simple model
        alpha = 0.1
        self.v += (u - self.v) * alpha
        return self.v