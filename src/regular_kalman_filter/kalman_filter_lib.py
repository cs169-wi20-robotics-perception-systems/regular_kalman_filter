import rospy

def propagation(state_estimate, change_in_state, prev_uncertainty_estimate, motion_noise):
    """
    Function that implements the propagation (prediction) step of a simple Kalman Filter.

    Args:
        state_estimate: The previous state estimate of the robot.
        change_in_state: The robot's estimate of its change in state.
        prev_uncertainty_estimate: The previous uncertainty estimate of the robot's state.
        motion_noise: The estimated noise of the robot's odometry based on motion.

    Returns:
        new_uncertainty_estimate: The new uncertainty estimate of the robot's state.
    """
    new_state_estimate = state_estimate + change_in_state
    # FROM SLIDES: Uncertainty estimate GROWS
    new_uncertainty_estimate = prev_uncertainty_estimate + motion_noise

    return new_state_estimate, new_uncertainty_estimate


def update(state_estimate, target_distance, scan_reading, uncertainty_estimate, sensor_noise):
    """
    Function that implements the update (correction) step of a simple Kalman Filter.

    Args:
        state_estimate: The current state estimate (distance) of the robot.
        target_distance: The initial measured distance of the robot from the wall.
        scan_reading: The current scan reading measurement.
        uncertainty_estimate: The current uncertainty estimate of the robot's state.
        sensor_noise: The estimated noise of the robot's sensor based either on the laser or on the rgbd camera.

    Returns:
        corrected_state_estimate: The corrected state estimate (distance) of the robot.
        corrected_uncertainty_estimate: The corrected uncertainty estimate of the robot's state.
    """
    actual_reading = target_distance - scan_reading
    rospy.logdebug("expected reading: " + str(state_estimate))
    rospy.logdebug("actual reading: " + str(actual_reading))

    # FROM SLIDES: Compute the difference between expected and "true"
    error = actual_reading - state_estimate
    rospy.logdebug("error: " + str(error))

    # FROM SLIDES: Compute covariance of sensor reading
    covariance = uncertainty_estimate + sensor_noise
    rospy.logdebug("covariance: " + str(covariance))

    # FROM SLIDES: Compute the Kalman Gain (how much to correct est.)
    kalman_gain = uncertainty_estimate / covariance
    rospy.logdebug("kalman gain: " + str(kalman_gain))

    # FROM SLIDES: Multiply residual times gain to correct state estimate
    corrected_state_estimate = state_estimate + kalman_gain * error
    rospy.logdebug("corrected state estimate: " + str(corrected_state_estimate))

    # FROM SLIDES: Uncertainty estimate SHRINKS
    corrected_uncertainty_estimate = uncertainty_estimate - uncertainty_estimate / covariance * uncertainty_estimate
    rospy.logdebug("corrected uncertainty estimate: " + str(corrected_uncertainty_estimate) + "\n")

    return corrected_state_estimate, corrected_uncertainty_estimate
