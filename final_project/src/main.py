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
        self.goals_list.print_goals()


if __name__ == "__main__":
    main = Main()
    main.operate_robot()
