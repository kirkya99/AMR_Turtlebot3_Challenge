#! /usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped


class MovebaseController:
    def __init__(self):
        rospy.loginfo("[MoveBaseController] Initializing action client")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_index = 0

        rospy.loginfo("[MoveBaseController] Waiting for action server")
        self.client.wait_for_server()

        rospy.loginfo("[MoveBaseController] Initializing action goal")
        self.goal = MoveBaseGoal()

    def feedback_callback(self, feedback):
        rospy.loginfo("[MoveBaseController] Goal feedback")

    def move_base(self, goals_list):
        self.current_index = 0
        while len(goals_list) > 0:
            target_pose = PoseStamped()

            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = goals_list[self.current_index].x
            target_pose.pose.position.y = goals_list[self.current_index].y
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

            result = self.client.get_result()
            if result:
                status = self.client.get_state()
                if status == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("[MovebaseController] Action succeeded: Robot reached the goal point")
                    # Perform action for successful completion
                    self.current_index = 0
                    del goals_list[self.current_index]

                    rospy.loginfo("[MovebaseController] Length: {0}".format(len(goals_list)))

                else:
                    rospy.logwarn("[MovebaseController] Action failed: Robot couldn't reach the goal point")
                    # Perform action for failure to reach the goal
                    self.current_index = self.current_index + 1
