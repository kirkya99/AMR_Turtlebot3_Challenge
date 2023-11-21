#! /usr/bin/env python3
import rospy
import yaml
import move_base_controller
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from copy import deepcopy


class Goal:
    # Initializes the goal with the relevant information.
    def __init__(self, x, y, orientation, reward, zone):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.reward = reward
        self.zone = zone

class GoalsList:
    # Initializes the goals list with the empty lists and reads the yaml-file for the receival of the goals list.
    def __init__(self):
        config_file_name = rospy.get_param("~final_goals_config_file", "final_goals.yaml")
        self.point_four = None
        self.point_six = None
        self.current_position = None

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        rospy.loginfo("[GoalsList] Configuration file name: " + str(config_file_name))
        path_to_open = config_file_name
        try:
            yaml_file = open(path_to_open, "r")
            self.configFile = yaml.load(yaml_file, yaml.SafeLoader)
        except Exception as _:
            rospy.logerr("[GoalsList] Could not open file " + str(path_to_open) + ".")
            rospy.logerr("[GoalsList] Exiting")
        self.easy_zone_list = []
        self.hard_zone_list = []
        self.combined_list = []

        self.move_base_controller = move_base_controller.MovebaseController()

        self.total_reward = 0

    # Callback function for the amcl_pose topic.
    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = Goal(position.x, position.y, orientation.z, 0, "none")

    # This function reads the goals from the goals yaml-file and saves them into the lists.
    def read_goals(self):
        try:
            goals_yaml = self.configFile["goals"]
        except Exception as _:
            goals_yaml = None
            raise Exception("[GoalsList] No tag 'goals' in yaml file found!")
        
        # The goals array from the yaml is iterated and the valuse are stored in a Goal instance.
        if goals_yaml is not None:
            # Iterate through goals
            for goal_key, goal_value in sorted(goals_yaml.items()):
                try:
                    rospy.logdebug("[GoalsList] " + str(goal_value))
                    self.add_point(goal_value['x'], goal_value['y'], goal_value['orientation'], goal_value['reward'],
                                   goal_value['zone'])

                except Exception as _:
                    rospy.logerr(
                        """[GoalsList] Could not add this point. Skipping it and continuing
                             with the next point"""
                    )

            self.point_six = deepcopy(self.hard_zone_list[0])

        else:
            raise Exception("[GoalsList] Could not open yaml file with goals listed!")

    # This function receives the goal information and creates an goal which will then be saved 
    # to the easy or hard zone list depending on the zone of the point. 
    def add_point(self, x, y, orientation, reward, zone):
        goal_point = Goal(x, y, orientation, reward, zone)
        if zone == 'easy':
            self.easy_zone_list.append(goal_point)
        elif zone == 'hard':
            self.hard_zone_list.append(goal_point)

    # Removes the given point from the easy zone list.
    def remove_easy_zone_point(self, index):
        del self.easy_zone_list[index]

    # Removes the given point from the hard zone list.
    def remove_hard_zone_point(self, index):
        del self.hard_zone_list[index]

    # This function has the task of managing the transfer of the goals of the easy zone list to the move_base_controller
    # depending of possible ways of moving the robot. If the path to an goal is blocked, the robot will navigate to the next goal. 
    # If the path is not blocked, it will always move towards the first point.
    def navigating_easy_zone(self):
        current_goal_index = 0
        self.sort_easy_zone_list()
        while len(self.easy_zone_list) > 0:
            status = self.move_base_controller.move_base(self.easy_zone_list[current_goal_index])
            # The goal was visited by the turtlebot
            if status is True:
                self.total_reward += self.easy_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Goal visited.")
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                self.remove_easy_zone_point(current_goal_index)
                current_goal_index = 0
                self.sort_easy_zone_list()
            # The goal or path to the goal was blocked
            else:
                self.goal_blocked_warning()
                if current_goal_index < len(self.easy_zone_list) - 1:
                    current_goal_index += 1
        rospy.loginfo("[GoalsList] All easy zone points visited.")


    # This function has the task of managing the transfer of the goals of the hard zone list to the move_base_controller
    # depending of possible ways of moving the robot. If the path to an goal is blocked, the robot will navigate to the next goal. 
    # If the path is not blocked, it will always move towards the first point.
    def navigating_hard_zone(self):
        current_goal_index = 0
        self.sort_hard_zone_list()
        while len(self.hard_zone_list) > 0:
            status = self.move_base_controller.move_base(self.hard_zone_list[current_goal_index])
            # The goal was visited by the turtlebot
            if status is True:
                self.total_reward += self.hard_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Goal visited.")
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                self.remove_hard_zone_point(current_goal_index)
                current_goal_index = 0
                self.sort_hard_zone_list()
            # The goal or path to the goal was blocked
            else:
                self.goal_blocked_warning()
                if current_goal_index < len(self.hard_zone_list) - 1:
                    current_goal_index += 1
        rospy.loginfo("[GoalsList] All hard zone points visited.")

    # Retrieves the sixth point for the manual control of the robot.
    def get_point_six(self):
        return self.point_six
    
    # Sorts the easy point list according to the current robot position.
    def sort_easy_zone_list(self):
        self.easy_zone_list.sort(key=self.get_distance_to_current)

    # Sorts the hard point list according the the current robot position.
    def sort_hard_zone_list(self):
        self.hard_zone_list.sort(key=self.get_distance_to_current)

    # Calculates the distance from the specificed goal to the current position.
    def get_distance_to_current(self, goal):
        return math.sqrt((goal.x - self.current_position.x)**2 + (goal.y - self.current_position.y)**2)
    
    # Print the goal blocked warning
    def goal_blocked_warning(self):
        rospy.logwarn("[GoalsList] Goal or path to goal was blocked")

    