def new_controller1(reference_t, observation_t, previous_pos, prev_reference, dt=1.0):
    # Compute the current error
    current_error = reference_t - observation_t
    
    # Compute the previous error based on the previous observation and reference
    previous_error = prev_reference - previous_pos[1]  # previous_pos[1] is the y-position (depth)
    
    # PD gains
    Kp = 0.12  # Proportional gain
    Kd = 0.7  # Derivative gain

    # Compute the derivative of the error
    error_derivative = (current_error - previous_error) / dt
    
    # PD Control law: proportional + derivative terms
    action = Kp * current_error + Kd * error_derivative

    # Log the control action
    print(f"Action: {action}, Error: {current_error}, Derivative: {error_derivative}")

    return action
