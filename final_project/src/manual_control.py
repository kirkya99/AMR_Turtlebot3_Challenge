#! /usr/bin/env python3
import rospy
import move_base_controller
from goals import Goal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


class ManualControl:
    # Initializes the ManualControl class.
    def __init__(self):
        self.infinity = 2.0
        self.move_base_controller = move_base_controller.MovebaseController()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.position = None
        self.orientation = None
        self.front = None  # Initialize front here
        self.left_front = None
        self.left = None
        self.left_aft = None
        self.right_front = None
        self.right = None
        self.right_aft = None
        self.twist = Twist()
        self.FORWARD = "FORWARD"
        self.IDLE = "IDLE"
        self.STATUS = self.FORWARD
        self.sensor_max_range = self.infinity

    # Callback function for the laser scan distance values.
    def laser_scan_callback(self, laser_scan):
        self.front = laser_scan.ranges[0]
        self.left_front = laser_scan.ranges[75]
        self.left = laser_scan.ranges[90]
        self.left_aft = laser_scan.ranges[105]
        self.right_front = laser_scan.ranges[-75]
        self.right = laser_scan.ranges[-90]
        self.right_aft = laser_scan.ranges[-105]

        if self.left_front > self.infinity or self.left_front > self.sensor_max_range:
            self.left_front = self.sensor_max_range
        if self.left > self.infinity or self.left > self.sensor_max_range:
            self.left = self.sensor_max_range
        if self.left_aft > self.infinity or self.left_aft > self.sensor_max_range:
            self.left_aft = self.sensor_max_range
        if self.right_front > self.infinity or self.right_front > self.sensor_max_range:
            self.right_front = self.sensor_max_range
        if self.right > self.infinity or self.right > self.sensor_max_range:
            self.right = self.sensor_max_range
        if self.right_aft > self.infinity or self.right_aft > self.sensor_max_range:
            self.right_aft = self.sensor_max_range

    # Callback function for the determination of the current position by amcl_pose.
    def amcl_pose_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    # Moves the robot forward with the speed of 0.1 m/s.
    def move_forward(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0

    # Turns the robot right with the speed of 0.25 m/s.
    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.25

    # Stops the angular and linear movements of the robot.
    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    # This function has the task of moving into the hard zone in three steps.
    # 1. The robot navigates to an helper goal directly infront of the sixth goal.
    # 2. The robot uses the orientation from amcl_pose to determinate if the goal is directly in front of it.
    #    If not, the robot will turn right until the goal is directly in front of it.
    # 3. The robot will move forward until the sixth goal is reached. The control is then returned to the goals_list.
    def enter_hard_zone(self, point_six):
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        rospy.loginfo("[ManualControl] Waiting for laser scan data...")
        while self.front is None:
            rospy.sleep(0.1)

        x = 1.4726425599703223
        y = -0.42066028037912384
        self.start_point = Goal(x, y, 1, 0, "none")
        self.end_point = Goal(point_six.x, point_six.y, 1, 0, "none")

        rospy.loginfo("[ManualControl] Point Start: x={}; y={}".format(self.start_point.x,self.start_point.y))
        rospy.loginfo("[ManualControl] Point End: x={}; y={}".format(self.end_point.x, self.end_point.y))
        rospy.loginfo("[ManualControl] -----")

        # Move towards starting point for entering the hard zone
        rospy.loginfo("[ManualControl] Move to starting point")
        status = False
        while status is False:
            status = self.move_base_controller.move_into_hard_zone()

        # Move forward until point six is reached
        while True:
            if self.check_orientation():
                self.stop_robot()
                break
            else:
                self.turn_right()

            self.vel_pub.publish(self.twist)

        while True:
            if self.position.x < self.end_point.x:
                self.move_forward()
            else:
                self.stop_robot()
                self.STATUS = self.IDLE
                break 

            self.vel_pub.publish(self.twist)


        rospy.loginfo("[ManualControl] End point is reached. Returning control to Main")

    # This goal checks if the orientation fits with the sixth goal. If the goal is directly in front of it, the robot will return true and if not
    def check_orientation(self):
        if self.orientation.w == 1:
            return True
        else:
            return False
