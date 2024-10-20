import numpy as np

class Controller():
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
        
        self.A = np.array([[1, 1], [0, 1]])
        self.B = np.array([[0], [1]])
        self.C = np.array([[1, 0]])
        self.D = np.array([[0]])

class PDController(Controller):
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray,
                 Kp: float = 0.15, Kd: float = 0.6):
        super().__init__(A, B, C, D)
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0

    def compute_control_action(self, x0: np.ndarray, reference: np.ndarray) -> np.ndarray:

        error = reference - self.C @ x0
        derivative = (error - self.previous_error) / self.A[1, 1]  # Assuming dt = 1 for simplicity
        
        u = self.Kp * error + self.Kd * derivative
        self.previous_error = error

        return u
