import rospy
import tf

import kalman_filter_lib

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

RATE = 10.0
SCAN_ANGLE_MIN_INDEX = 5 # pi/12
SCAN_ANGLE_MAX_INDEX = 354 # -pi/12

class Rosbot:
    def __init__(self, robot_sensor, scan_sensor, target_distance=2.0):
        self.rate = rospy.Rate(RATE)  # 10 Hz
        self.scan_sensor = scan_sensor
        self.target_distance = target_distance

        self.scan_reading = 0.0
        self.prev_pose_reading = 0.33    # Only used when subscribing to pose topic
        self.state_estimate = 0.0

        self.uncertainty_estimate = 2000.0
        self.motion_noise = 50.0
        self.sensor_noise = 200.0

        self.br = tf.TransformBroadcaster()

        self.pose_kf_pub = rospy.Publisher("pose_kf", PoseWithCovarianceStamped, queue_size=1)

        if 1 == robot_sensor:
            rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)
            rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)

        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.wait_for_message("scan", LaserScan)

        # use /cmd_vel as robot's estimate
        if 0 == robot_sensor:
            rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        # use /pose as robot's estimate
        if 1 == robot_sensor:
            rospy.Subscriber("pose", PoseStamped, self.pose_callback)



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
                    avg_scan_distance = avg_scan_distance + msg.ranges[index]
                    num_scan_points = num_scan_points + 1

        avg_scan_distance = avg_scan_distance / num_scan_points
        self.scan_reading = avg_scan_distance

        if 1 == self.scan_sensor:
            self.kalman_filter()


    def initial_pose_callback(self, msg):
        """
        Callback for retrieving the robot's initial state.

        Args:
            msg: PoseWithCovarianceStamped msg that has the current pose, orientation, and covariance matrix.
        """
        self.prev_pose_reading = msg.pose.pose.position.x


    def cmd_vel_callback(self, msg):
        """
        Callback for retrieving the robot's linear velocity. Then implements a step in the Kalman Filter.

        Args:
            msg: Twist msg that has the current robot's linear and angular velocity.
        """
        change_in_state = (1 / RATE) * msg.linear.x

        self.rate.sleep()

        self.state_estimate = self.state_estimate + change_in_state

        if 0 == self.scan_sensor:
            self.kalman_filter()


    def pose_callback(self, msg):
        """
        Callback for retrieving the robot's position. Then implements a step in the Kalman Filter.

        Args:
            msg: PoseStamped msg that has the current robot's position and orientation.
        """
        change_in_state = msg.pose.position.x - self.prev_pose_reading
        self.prev_pose_reading = msg.pose.position.x
        self.state_estimate = self.state_estimate + change_in_state

        if 0 == self.scan_sensor:
            self.kalman_filter()


    def kalman_filter(self):
        """
        Function that implements a simple Kalman Filter. Then publishes the results as a ROS topic and as a transformation.

        Args:
            change_in_state: The change in state (distance) the robot estimates it has made.
        """
        # Updates the robot's estimated uncertainty using either linear velocity or odometry data.
        self.uncertainty_estimate = kalman_filter_lib.propagation(self.uncertainty_estimate, self.motion_noise)

        # Updates the robot's estimated state and uncertainty using sensor data and a simple Kalman Filter.
        self.state_estimate, self.uncertainty_estimate = kalman_filter_lib.update(self.state_estimate, self.target_distance,
            self.scan_reading, self.uncertainty_estimate, self.sensor_noise)

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
