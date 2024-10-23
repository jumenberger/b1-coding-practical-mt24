class Controller:
    def control(self, K_p, K_d, error_t, prev_error, t):
        # denotes error between reference r[t] and output (depth) y[t], at time t.
        if t > 0:
            action = K_p * error_t + K_d * (error_t - prev_error)
        else:
            action = K_p * error_t

        return action
