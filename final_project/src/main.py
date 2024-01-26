#! /usr/bin/env python3
import rospy
import goals as points


class Main:
    def __init__(self):
        rospy.loginfo(
            "[Main] Initializing ROS Node"
        )
        rospy.init_node('final_project')
        self.goals_list = points.GoalsList()

    def operate_robot(self):
        self.goals_list.read_goals()
        rospy.loginfo("[Main] ---------------------")
        self.goals_list.navigating_easy_zone()
        # TODO: Add follow wall for the passage between the two walls

        # TODO: Return control to the move_base_controller for the goals in the hard zone


if __name__ == "__main__":
    main = Main()
    main.operate_robot()
