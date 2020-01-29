import rospy
import tf

import kalman_filter_lib

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

class Rosbot:
    def __init__(self, robot_sensor, scan_sensor, target_distance=2.0, rate=10,
                scan_angle_min_index=5, scan_angle_max_index=354,
                uncertainty_estimate=2000.0, motion_noise=50.0, sensor_noise=200.0):
        self.HZ = rate
        self.rate = rospy.Rate(self.HZ)  # 10 Hz

        self.scan_sensor = scan_sensor
        self.scan_reading = 0.0
        self.SCAN_ANGLE_MIN_INDEX = scan_angle_min_index    # pi/12
        self.SCAN_ANGLE_MAX_INDEX = scan_angle_max_index    # -pi/12

        self.target_distance = target_distance

        self.prev_pose_reading = 0.0    # Only used when subscribing to pose topic
        self.state_estimate = 0.0

        self.uncertainty_estimate = uncertainty_estimate
        self.MOTION_NOISE = motion_noise
        self.SENSOR_NOISE = sensor_noise

        self.br = tf.TransformBroadcaster()

        self.pose_kf_pub = rospy.Publisher("pose_kf", PoseWithCovarianceStamped, queue_size=1)

        # If using pose topic, then wait for the intial pose
        if 1 == robot_sensor:
            rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)
            rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)

        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.wait_for_message("scan", LaserScan)

        # Use /cmd_vel as robot's estimate
        if 0 == robot_sensor:
            rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        # Use /pose as robot's estimate
        if 1 == robot_sensor:
            rospy.Subscriber("pose", PoseStamped, self.pose_callback)



    def scan_callback(self, msg):
        """
        Callback for retrieving the laser scan data. Use a range (laser) or all of the data (rgbd camera), depending
        on type of sensor, and calculate one scan reading measurement.

        Calls the kalman filter (update) function.

        Args:
            msg: LaserScan msg that has a list of scan measurements.
        """
        # Check if using the correct laser scan data
        if ((0 == self.scan_sensor and "base_link" == msg.header.frame_id) or
            (1 == self.scan_sensor and "laser" == msg.header.frame_id)):
            return

        # Calculate average scan reading
        avg_scan_distance = 0.0
        num_scan_points = 0.0

        # Use laser sensor data
        if "laser" == msg.header.frame_id:
            # Use data between SCAN_ANGLE_MIN_INDEX and SCAN_ANGLE_MAX_INDEX
            for index in range(self.SCAN_ANGLE_MIN_INDEX):
                if msg.ranges[index] < msg.range_max:
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1

            for index in range(self.SCAN_ANGLE_MAX_INDEX, 360):
                if msg.ranges[index] < msg.range_max:
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1
        # Use rgbd camera data
        else:
            # Use all data
            for index in range(len(msg.ranges)):
                if msg.ranges[index] < msg.range_max:
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1

        avg_scan_distance = avg_scan_distance / num_scan_points
        self.scan_reading = avg_scan_distance

        self.kalman_filter()


    def initial_pose_callback(self, msg):
        """
        Callback for retrieving the robot's initial state.

        Args:
            msg: PoseWithCovarianceStamped msg that has the current pose, orientation, and covariance matrix.
        """
        self.prev_pose_reading = msg.pose.pose.position.x

        rospy.logdebug("initial pose: " + str(self.prev_pose_reading) + "\n")


    def cmd_vel_callback(self, msg):
        """
        Callback for retrieving the robot's linear velocity. Then calls the propoagation step in the Kalman filter.

        Args:
            msg: Twist msg that has the current robot's linear and angular velocity.
        """
        change_in_state = (1 / self.HZ) * msg.linear.x

        # Sleep to make sure the robot has moved at the predicted change in state.
        self.rate.sleep()

        # Updates the robot's estimated uncertainty using either linear velocity or odometry data.
        self.state_estimate, self.uncertainty_estimate = kalman_filter_lib.propagation(self.state_estimate, change_in_state, self.uncertainty_estimate, self.MOTION_NOISE)


    def pose_callback(self, msg):
        """
        Callback for retrieving the robot's position. Then calls the propoagation step in the Kalman filter.

        Args:
            msg: PoseStamped msg that has the current robot's position and orientation.
        """
        change_in_state = msg.pose.position.x - self.prev_pose_reading
        self.prev_pose_reading = msg.pose.position.x
        #self.state_estimate = self.state_estimate + change_in_state

        # Updates the robot's estimated uncertainty using either linear velocity or odometry data.
        self.state_estimate, self.uncertainty_estimate = kalman_filter_lib.propagation(self.state_estimate, change_in_state, self.uncertainty_estimate, self.MOTION_NOISE)


    def kalman_filter(self):
        """
        Function that implements a simple Kalman Filter. Then publishes the results as a ROS topic and as a transformation.
        """
        # Updates the robot's estimated state and uncertainty using sensor data and a simple Kalman Filter.
        self.state_estimate, self.uncertainty_estimate = kalman_filter_lib.update(self.state_estimate, self.target_distance,
            self.scan_reading, self.uncertainty_estimate, self.SENSOR_NOISE)

        # Publish the estimated pose calculated using a simple Kalman Filter.
        pose_kf_msg = PoseWithCovarianceStamped()
        pose_kf_msg.header.stamp = rospy.Time.now()
        pose_kf_msg.header.frame_id = "base_link"
        pose_kf_msg.pose.pose.position.x = self.state_estimate
        pose_kf_msg.pose.covariance[0] = self.uncertainty_estimate
        self.pose_kf_pub.publish(pose_kf_msg)

        # Publish a transformation based on the estimated pose calculated using a simple Kalman Filter.
        self.br.sendTransform((self.state_estimate, 0, 0),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),
                                "odom_kf",
                                "base_footprint")
