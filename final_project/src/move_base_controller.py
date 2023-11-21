#! /usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class MovebaseController:
    # Initializes the MovebaseController
    def __init__(self):
        rospy.loginfo("[MoveBaseController] Initializing action client")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()

        rospy.loginfo("[MoveBaseController] Waiting for action server")
        self.client.wait_for_server()

        rospy.loginfo("[MoveBaseController] Initializing action goal")
        self.duration = 30

    # The callback of the feedback function which prints the results as debug print.
    def feedback_callback(self, feedback):
        rospy.logdebug(feedback)

    # The function which sends the received goal to the move_base action server.
    # If the goal is reached, which means that the position of the received goal is reached within the time limit, 
    # the function returns true so that this goal can be deleted from the list and the next goal selected.
    # But if the goal was not reached the function returns false and the goal will not be deleted and another goal will be selected.
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

        rospy.sleep(2)

        rospy.loginfo("[MovebaseController] Sending goal to action server")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        wait = self.client.wait_for_result(rospy.Duration(self.duration))

        if wait is True:
            rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
            return True
        else:
            rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
            return False

    # This function has the task of navigating the robot to the helper goal for the maneuvering of the robot into the hard zone.
    def move_into_hard_zone(self):
        rospy.loginfo("[MoveBaseController] Initializing next navigation point")
        target_pose = PoseStamped()

        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 1.24827112437622
        target_pose.pose.position.y = -0.66988540757462
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        
        self.goal.target_pose = target_pose

        rospy.sleep(2)
        rospy.loginfo("[MovebaseController] Sending goal to action server")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        wait = self.client.wait_for_result(rospy.Duration(self.duration))

        if wait is True:
            rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
            return True
        else:
            rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
            return False
        