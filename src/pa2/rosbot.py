import rospy
import tf

import kalman_filter_lib

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

RATE = 10.0
SCAN_ANGLE_MIN_INDEX = 15 # pi/12
SCAN_ANGLE_MAX_INDEX = 345 # -pi/12

class Rosbot:
    def __init__(self, robot_sensor, scan_sensor, distance_from_wall=2.0):
        self.rate = rospy.Rate(RATE)  # 10 Hz
        self.scan_sensor = scan_sensor
        self.target = distance_from_wall

        self.scan_reading = 1.20
        self.state_estimate = 0.0
        self.initial_state = PoseWithCovarianceStamped()

        self.uncertainty_estimate = 1000.0
        self.wheel_noise = 1.0
        self.sensor_noise = 1.0

        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)

        # use /cmd_vel as robot's estimate
        if 0 == robot_sensor:
            rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        # use /pose as robot's estimate
        if 1 == robot_sensor:
            rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.pose_kf_pub = rospy.Publisher("/pose_kf", PoseWithCovarianceStamped, queue_size=1)


    def scan_callback(self, msg):
        """
        Callback for retrieving the laser scan data. Use a range (laser) or all of the data (rgbd camera), depending
        on type of sensor, and calculate one scan reading measurement.

        Args:
            msg: LaserScan msg that has a list of scan measurements.
        """
        # Check if using the correct laser scan data
        if ((0 == self.scan_sensor and "base_link" == msg.header.frame_id) or
            (1 == self.scan_sensor and "laser" == msg.header.frame_id)):
            return

        avg_scan_distance = 0.0
        num_scan_points = 0.0

        # Use laser sensor data
        if "laser" == msg.header.frame_id:
            # Use data between SCAN_ANGLE_MIN_INDEX and SCAN_ANGLE_MAX_INDEX
            for index in range(SCAN_ANGLE_MIN_INDEX):
                if msg.ranges[index] < msg.range_max:
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1

            for index in range(SCAN_ANGLE_MAX_INDEX, 360):
                if msg.ranges[index] < msg.range_max:
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1
        # Use rgbd camera data
        else:
            # Use all data
            for index in range(len(msg.ranges)):
                if msg.ranges[index] < msg.range_max:
                    print(msg.ranges[index])
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1

        avg_scan_distance = avg_scan_distance / num_scan_points
        self.scan_reading = avg_scan_distance


    def initial_pose_callback(self, msg):
        """
        Callback for retrieving the robot's initial state.

        Args:
            msg: PoseWithCovarianceStamped msg that has the current pose, orientation, and covariance matrix.
        """
        self.initial_state = msg


    def cmd_vel_callback(self, msg):
        """
        Callback for retrieving the robot's linear velocity. Then implements a step in the Kalman Filter.

        Args:
            msg: Twist msg that has the current robot's linear and angular velocity.
        """
        change_in_state = (1 / RATE) * msg.linear.x

        self.kalman_filter(change_in_state)


    def pose_callback(self, msg):
        """
        Callback for retrieving the robot's position. Then implements a step in the Kalman Filter.

        Args:
            msg: PoseStamped msg that has the current robot's position and orientation.
        """
        change_in_state = msg.pose.position.x

        self.kalman_filter(change_in_state)


    def kalman_filter(self, change_in_state):
        """
        Function that implements a simple Kalman Filter. Then publishes the results as a ROS topic and as a transformation.

        Args:
            change_in_state: The change in state (distance) the robot estimates it has made.
        """
        # Updates the robot's estimated uncertainty using either wheel velocity or odometry data.
        self.state_estimate, self.uncertainty_estimate = kalman_filter_lib.propagation(self.state_estimate, change_in_state,
            self.uncertainty_estimate, self.wheel_noise)

        # Updates the robot's estimated state and uncertainty using sensor data and a simple Kalman Filter.
        self.state_estimate, self.uncertainty_estimate, covariance = kalman_filter_lib.update(self.state_estimate, self.target, self.scan_reading,
            self.uncertainty_estimate, self.sensor_noise)

        # TO DO: remove when scan readings are accurate again
        self.scan_reading = self.scan_reading - 0.015

        # Publish the estimated pose calculated using a simple Kalman Filter.
        pose_kf_msg = PoseWithCovarianceStamped()
        pose_kf_msg.header.stamp = rospy.Time.now()
        pose_kf_msg.header.frame_id = "base_link"
        pose_kf_msg.pose.pose.position.x = self.state_estimate
        pose_kf_msg.pose.covariance = [covariance, 0, 0, 0, 0, 0,
                                        0, 1, 0, 0, 0, 0,
                                        0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 1, 0, 0,
                                        0, 0, 0, 0, 1, 0,
                                        0, 0, 0, 0, 0, 1]
        self.pose_kf_pub.publish(pose_kf_msg)

        # Publish a transformation based on the estimated pose calculated using a simple Kalman Filter.
        self.br.sendTransform((self.state_estimate, 0, 0),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),
                                "odom_kf",
                                "base_footprint")
