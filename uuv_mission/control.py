class PDController:
    def __init__(self, k_p, k_d):
        self.k_p = k_p
        self.k_d = k_d
        self.previous_error = 0

    def __call__(self, reference, observation):
        error = reference - observation
        action = self.k_p * error + self.k_d * (error - self.previous_error)
        self.previous_error = error
        return action