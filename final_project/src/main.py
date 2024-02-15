#! /usr/bin/env python3
import rospy
import goals as points
import manual_control


class Main:
    def __init__(self):
        rospy.loginfo(
            "[Main] Initializing ROS Node"
        )
        rospy.init_node('final_project')
        self.goals_list = points.GoalsList()
        # rospy.loginfo("[MAIN] Point 4: x={}; y={}", point_four.x, point_four.y)
        # rospy.loginfo("[MAIN] Point 6: x={}; y={}", point_six.x, point_six.y)
        self.manual_control = manual_control.ManualControl()

    def operate_robot(self):
        self.goals_list.read_goals()
        rospy.loginfo("[Main] ---------------------")
        self.goals_list.navigating_easy_zone()
        # self.manual_control.enter_hard_zone(self.goals_list.get_point_four(), self.goals_list.get_point_six())        
        self.goals_list.navigating_hard_zone()


if __name__ == "__main__":
    main = Main()
    main.operate_robot()
