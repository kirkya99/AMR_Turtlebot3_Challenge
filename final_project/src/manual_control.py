#! /usr/bin/env python3
import rospy
import move_base_controller
from goals import Goal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


class ManualControl:
    def __init__(self, point_position):
        self.infinity = 2.0
        self.move_base_controller = move_base_controller.MovebaseController()
        self.wall_position = Goal(0.0, 0.0, 1, 0, "none")
        self.opening_position = Goal(0.0, 0.0, 1, 0, "none")
        self.point_position = point_position
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.position = None
        self.orientation = None
        self.twist = Twist()
        self.FORWARD = "FORWARD"
        self.LEFT="LEFT"
        self.RIGHT="RIGHT"
        self.IDLE = "IDLE"
        self.STATUS = self.FORWARD


    def laser_scan_callback(self, laser_scan):
        self.min_distance = min(laser_scan)
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
        self.orientation = msg.pose.orientation
        rospy.loginfo("Position: x={}, y={}, z={}".format(self.position.x, self.position.y, self.position.z))
        rospy.loginfo(
            "Orientation: x={}, y={}, z={}, w={}".format(self.orientation.x, self.orientation.y, self.orientation.z,
                                                         self.orientation.w))

    def point_reached(self):
        if self.position.x > self.point_position.x and self.position.y > self.point_position.y:
            return True
        else:
            return False


    def moveForward(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0


    def moveLeft(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.15

    def moveRight(self):
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.15

    def stopRobot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def move_into_hard_zone(self):
        self.move_base_controller.move_base(self.position)

        while not rospy.is_shutdown() and self.STATUS != self.IDLE:
            if self.min_distance < 0.25:
                self.stopRobot()
            elif self.STATUS == self.RIGHT:
                if self.right_front < self.right_aft or self.right_aft == 0 or self.right_front == 0:
                    self.moveRight()
                else:
                    self.stopRobot()
                    self.STATUS = self.FORWARD
            elif self.STATUS == self.FORWARD:
                self.moveForward()
                if self.position.y >= self.point_position.y:
                    self.STATUS = self.LEFT
                elif self.position.x >= self.point_position.x:
                    self.STATUS = self.IDLE
                elif self.position.y < self.point_position.y:
                    self.moveForward()
                    self.STATUS = self.FORWARD
                elif self.position.x < self.point_position.x:
                    self.STATUS = self.FORWARD
                    self.moveForward()                
            elif self.STATUS == self.LEFT:
                if self.left_front < self.left_aft or self.left_aft == 0 or self.left_front == 0:
                    self.moveLeft()
                else:
                    self.stopRobot()
                    self.STATUS = self.FORWARD
            
            self.vel_pub.publish(self.twist)

        self.stopRobot()
        self.vel_pub.publish(self.twist)
    

    def move_into_hard_zone_v2(self):
        self.move_base_controller.move_base(self.position)

        while not rospy.is_shutdown() and self.STATUS != self.IDLE:
            if self.min_distance < 0.25:
                self.stopRobot()
            elif self.STATUS == self.RIGHT:
                if self.right_front < self.right_aft or self.right_aft == 0 or self.right_front == 0:
                    self.moveRight()
                else:
                    self.stopRobot()
                    self.STATUS = self.FORWARD
            elif self.STATUS == self.FORWARD:
                self.moveForward()
                if self.position.y >= self.point_position.y:
                    self.STATUS = self.LEFT
                    self.stopRobot()
                elif self.position.x >= self.point_position.x:
                    self.STATUS = self.IDLE
                    self.stopRobot()
            elif self.STATUS == self.LEFT:
                if self.left_front < self.left_aft or self.left_aft == 0 or self.left_front == 0:
                    self.moveLeft()
                else:
                    self.stopRobot()
                    self.STATUS = self.FORWARD
            
            self.vel_pub.publish(self.twist)

        self.stopRobot()
        self.vel_pub.publish(self.twist)

