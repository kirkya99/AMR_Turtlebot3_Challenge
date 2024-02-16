#! /usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped


class MovebaseController:
    def __init__(self):
        rospy.loginfo("[MoveBaseController] Initializing action client")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()

        rospy.loginfo("[MoveBaseController] Waiting for action server")
        self.client.wait_for_server()

        rospy.loginfo("[MoveBaseController] Initializing action goal")
        self.duration = 30

    def feedback_callback(self, feedback):
        rospy.logdebug(feedback)

    def move_base(self, goal):
        rospy.loginfo("[MoveBaseController] Initializing next navigation point")
        target_pose = PoseStamped()

        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = goal.x
        target_pose.pose.position.y = goal.y
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        self.goal.target_pose = target_pose

        rospy.loginfo("[MoveBaseController] Waiting for message")
        rospy.sleep(2)

        rospy.loginfo("[MovebaseController] Sending goal")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        rospy.loginfo("[MovebaseController] Wait for result")
        wait = self.client.wait_for_result(rospy.Duration(self.duration))

        if wait is True:
            rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
            return True
        else:
            rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
            return False


    def move_into_hard_zone(self):
        rospy.loginfo("[MoveBaseController] Initializing next navigation point")
        target_pose = PoseStamped()

        x_new = 1.24827112437622
        y_new = -0.66988540757462
        x_old = 1.8000541357545716
        y_old = -0.44184322825998795

        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x_new
        target_pose.pose.position.y = y_new
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1

        self.goal.target_pose = target_pose

        rospy.loginfo("[MoveBaseController] Waiting for message")
        rospy.sleep(2)

        rospy.loginfo("[MovebaseController] Sending goal")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        rospy.loginfo("[MovebaseController] Wait for result")
        wait = self.client.wait_for_result(rospy.Duration(self.duration))

        if wait is True:
            rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
            return True

        else:
            rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
            return False
        