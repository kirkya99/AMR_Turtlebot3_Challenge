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

        # if hasattr(goal, 'reward'):
        #     rospy.loginfo("[MoveBaseController] Reward: %s", str(goal.reward))
        # else:
        #     rospy.logwarn("[MoveBaseController] 'reward' attribute not found in goal")

        self.goal.target_pose = target_pose

        rospy.loginfo("[MoveBaseController] Waiting for message")
        rospy.sleep(2)

        rospy.loginfo("[MovebaseController] Sending goal")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        rospy.loginfo("[MovebaseController] Wait for result")
        wait = self.client.wait_for_result(rospy.Duration(self.duration))


        # result = self.client.get_result()
        # if result:
        # status = self.client.get_state()
        if wait is True:
            rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
            # Perform action for successful completion
            return True

        else:
            rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
            # Perform action for failure to reach the goal
            return False
        # return True
        