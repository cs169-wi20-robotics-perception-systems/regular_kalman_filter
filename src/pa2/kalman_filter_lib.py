def propagation(prev_state_estimate, change_in_state, prev_uncertainty_estimate, wheel_noise):
    """
    Function that implements the propagation (prediction) step of a simple Kalman Filter.

    Args:
        prev_state_estimate: The previous state estimate (distance) of the robot.
        change_in_state: The change in state (distance) the robot has estimated it made.
        prev_uncertainty_estimate: The previous uncertainty estimate of the robot's state.
        wheel_noise: The estimated noise of the robot's odometry based on the wheels' movements.

    Returns:
        new_state_estimate: The new state estimate (distance) of the robot.
        new_uncertainty_estimate: The new uncertainty estimate of the robot's state.
    """
    # FROM SLIDES: State estimate is updated from system dynamics
    new_state_estimate = prev_state_estimate + change_in_state
    # FROM SLIDES: Uncertainty estimate GROWS
    new_uncertainty_estimate = prev_uncertainty_estimate + wheel_noise

    return new_state_estimate, new_uncertainty_estimate


def update(state_estimate, target, scan_reading, uncertainty_estimate, sensor_noise):
    """
    Function that implements the update (correction) step of a simple Kalman Filter.

    Args:
        state_estimate: The current state estimate (distance) of the robot.
        scan_distance: The current scan reading measurement.
        uncertainty_estimate: The current uncertainty estimate of the robot's state.
        sensor_noise: The estimated noise of the robot's sensor based either on the laser or on the rgbd camera.

    Returns:
        corrected_state_estimate: The corrected state estimate (distance) of the robot.
        corrected_uncertainty_estimate: The corrected uncertainty estimate of the robot's state.
    """
    # FROM SLIDES: Compute expected value of sensor reading
    expected_reading = target - state_estimate
    # FROM SLIDES: Compute the difference between expected and "true"
    error = scan_reading - expected_reading
    # FROM SLIDES: Compute covariance of sensor reading
    covariance = uncertainty_estimate + sensor_noise
    # FROM SLIDES: Compute the Kalman Gain (how much to correct est.)
    kalman_gain = uncertainty_estimate / covariance
    # FROM SLIDES: Multiply residual times gain to correct state estimate
    corrected_state_estimate = state_estimate + kalman_gain * error
    # FROM SLIDES: Uncertainty estimate SHRINKS
    corrected_uncertainty_estimate = uncertainty_estimate - uncertainty_estimate / covariance * uncertainty_estimate

    return corrected_state_estimate, corrected_uncertainty_estimate, covariance
