import numpy as np
import cvxpy as cp

class Controller:
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray):
        
        self.A = A
        self.B = B
        self.C = C
        self.D = D

class PDController(Controller):
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray,
                 Kp: float = 0.15, Kd: float = 0.6):

        super().__init__(A, B, C, D)
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0.

    def compute_control_action(self, x0: np.ndarray, reference: float) ->float:

        error = reference - self.C @ x0
        derivative = (error - self.previous_error)
        
        u = self.Kp * error + self.Kd * derivative
        self.previous_error = error

        return u.item()
    
class MPCController(Controller):
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray,
                 horizon: int, Q: np.ndarray, R: np.ndarray):

        super().__init__(A, B, C, D)
        self.horizon = horizon
        self.Q = Q
        self.R = R

    def compute_control_action(self, x0: np.ndarray, reference: np.ndarray) -> np.ndarray:

        x = cp.Variable((self.A.shape[0], self.horizon + 1))
        u = cp.Variable((self.B.shape[1], self.horizon))
        y = cp.Variable((self.C.shape[0], self.horizon + 1))

        cost = 0
        constraints = [x[:, 0] == x0]
        for t in range(self.horizon):
            cost += cp.quad_form(y[:, t] - reference[:, t], self.Q) + cp.quad_form(u[:, t], self.R)
            constraints += [
                x[:, t + 1] == self.A @ x[:, t] + self.B @ u[:, t],
                y[:, t] == self.C @ x[:, t] + self.D @ u[:, t]
            ]

        problem = cp.Problem(cp.Minimize(cost), constraints)

        problem.solve()

        return u[:, 0].value # return first control action