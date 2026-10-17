class Controller:

    def __init__(self):
        self.previous_error = 0

    def control(self, reference_depth, current_depth):

        error = reference_depth - current_depth

        control_action = (
            0.15 * error
            + 0.6 * (error - self.previous_error)
        )

        self.previous_error = error

        return control_action