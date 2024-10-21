class Controller:
    def __init__(self):
        pass

    def get_action(self, mission, t, observation_t):
        # You are required to implement this method
        Kp = 0.15
        Kd = 0.6
        error = mission.reference[t] - observation_t
        error_dot = mission.reference[t] - mission.reference[t-1]
        return Kp * error + Kd * error_dot