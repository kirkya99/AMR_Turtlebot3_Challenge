#! /usr/bin/env python3
import rospy
import goals as points
import move_base_controller


class Main:
    def __init__(self):
        rospy.loginfo(
            "[Main] Initializing ROS Node"
        )
        rospy.init_node('final_project')
        self.goals_list = points.GoalsList()
        self.move_base_controller = move_base_controller.MovebaseController()

    def operate_robot(self):
        self.goals_list.read_goals()
        rospy.loginfo("[Main] ---------------------")
        self.goals_list.print_goals()
        goals_list = self.goals_list.get_easy_zone_list()
        self.move_base_controller.move_base(goals_list)
        # TODO: Add follow wall for the passage between the two walls

        # TODO: Return control to the move_base_controller for the goals in the hard zone


if __name__ == "__main__":
    main = Main()
    main.operate_robot()
