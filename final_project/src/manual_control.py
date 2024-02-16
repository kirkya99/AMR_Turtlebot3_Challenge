#! /usr/bin/env python3
import rospy
import move_base_controller
from goals import Goal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


class ManualControl:
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

        
    def laser_scan_callback(self, laser_scan):
        # self.min_distance_front = min(laser_scan.ranges[-45:45])
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

    def amcl_pose_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def move_forward(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.25

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def enter_hard_zone(self, point_four, point_six):
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        rospy.loginfo("[ManualControl] Waiting for laser scan data...")
        while self.front is None:
            rospy.sleep(0.1)

        rospy.loginfo("[ManualControl] -----")
        rospy.loginfo("[ManualControl] Point 4: x={}; y={}".format(point_four.x,point_four.y))
        rospy.loginfo("[ManualControl] Point 6: x={}; y={}".format(point_six.x, point_six.y))
        rospy.loginfo("[ManualControl] -----")

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
        # rospy.loginfo("[ManualControl] Move forward until the end point is reached")

        self.move_base_controller.move_base(self.end_point)

        # while True:
        #     if self.check_orientation():
        #         self.stop_robot()
        #         break
        #     else:
        #         self.turn_right()

        #     self.vel_pub.publish(self.twist)

        # while True:
        #     if self.position.x < self.end_point.x:
        #         self.move_forward()
        #     else:
        #         self.stop_robot()
        #         self.STATUS = self.IDLE
        #         break 

        #     self.vel_pub.publish(self.twist)


        rospy.loginfo("[ManualControl] End point is reached. Returning control to Main")


    def check_orientation(self):
        if self.orientation.w == 1:
            return True
        else:
            return False
