import rospy

def propagation(prev_uncertainty_estimate, motion_noise):
    """
    Function that implements the propagation (prediction) step of a simple Kalman Filter.

    Args:
        prev_state_estimate: The previous state estimate (distance) of the robot.
        change_in_state: The change in state (distance) the robot has estimated it made.
        prev_uncertainty_estimate: The previous uncertainty estimate of the robot's state.
        motion_noise: The estimated noise of the robot's odometry based on motion.

    Returns:
        new_state_estimate: The new state estimate (distance) of the robot.
        new_uncertainty_estimate: The new uncertainty estimate of the robot's state.
    """
    # FROM SLIDES: Uncertainty estimate GROWS
    new_uncertainty_estimate = prev_uncertainty_estimate + motion_noise

    return new_uncertainty_estimate


def update(state_estimate, target_distance, scan_reading, uncertainty_estimate, sensor_noise):
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
    actual_reading = target_distance - scan_reading
    rospy.logdebug("expected_reading: " + str(state_estimate))
    rospy.logdebug("actual_reading: " + str(actual_reading))

    # FROM SLIDES: Compute the difference between expected and "true"
    error = actual_reading - state_estimate
    rospy.logdebug("error: " + str(error))

    # FROM SLIDES: Compute covariance of sensor reading
    covariance = uncertainty_estimate + sensor_noise
    rospy.logdebug("covariance: " + str(covariance))

    # FROM SLIDES: Compute the Kalman Gain (how much to correct est.)
    kalman_gain = uncertainty_estimate / covariance
    rospy.logdebug("kalman_gain: " + str(kalman_gain))

    # FROM SLIDES: Multiply residual times gain to correct state estimate
    corrected_state_estimate = state_estimate + kalman_gain * error
    rospy.logdebug("corrected_state_estimate: " + str(corrected_state_estimate))

    # FROM SLIDES: Uncertainty estimate SHRINKS
    corrected_uncertainty_estimate = uncertainty_estimate - uncertainty_estimate / covariance * uncertainty_estimate
    rospy.logdebug("corrected_uncertainty_estimate: " + str(corrected_uncertainty_estimate))

    rospy.logdebug("sec: " + str(rospy.Time.now().secs))
    rospy.logdebug("nsec: " + str(rospy.Time.now().nsecs) + "\n")

    return corrected_state_estimate, corrected_uncertainty_estimate
