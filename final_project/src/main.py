#! /usr/bin/env python3
import rospy
import goals as points
import manual_control


class Main:
    # Initializes the main class
    def __init__(self):
        rospy.loginfo(
            "[Main] Initializing ROS Node"
        )
        rospy.init_node('final_project')
        self.goals_list = points.GoalsList()
        self.manual_control = manual_control.ManualControl()

    # This function is the main function for the robot operations.
    def operate_robot(self):
        self.goals_list.read_goals()
        rospy.loginfo("[Main] ---------------------")

        duration = rospy.Duration(3)
        rospy.sleep(duration)

        self.goals_list.navigating_easy_zone()
        self.manual_control.enter_hard_zone(self.goals_list.get_point_six())        
        self.goals_list.navigating_hard_zone()

# Creates an instance of Main and calls the operate_robot() function.
if __name__ == "__main__":
    main = Main()
    main.operate_robot()
