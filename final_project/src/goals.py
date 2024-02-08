#! /usr/bin/env python3
import rospy
import yaml
import move_base_controller
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from copy import deepcopy


class Goal:
    def __init__(self, x, y, orientation, reward, zone):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.reward = reward
        self.zone = zone

class GoalsList:
    def __init__(self):
        config_file_name = rospy.get_param("~goals_config_file", "goals.yaml")
        # config_file_name = rospy.get_param("~final_goals_config_file", "final_goals.yaml")
        # config_file_name = rospy.get_param("~demo_goals_config_file", "demo_goals.yaml")
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

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = Goal(position.x, position.y, orientation.z, 0, "none")

    def read_goals(self):
        try:
            goals_yaml = self.configFile["goals"]
        except Exception as _:
            goals_yaml = None
            raise Exception("[GoalsList] No tag 'goals' in yaml file found!")

        if goals_yaml is not None:
            # Iterate through goals
            for goal_key, goal_value in sorted(goals_yaml.items()):
                try:
                    rospy.loginfo("[GoalsList] " + str(goal_value))
                    self.add_point(goal_value['x'], goal_value['y'], goal_value['orientation'], goal_value['reward'],
                                   goal_value['zone'])

                except Exception as _:
                    rospy.logerr(
                        """[GoalsList] Could not add this point. Skipping it and continuing
                             with the next point"""
                    )

            self.point_four = deepcopy(self.easy_zone_list[3])
            self.point_six = deepcopy(self.hard_zone_list[0])

            rospy.loginfo("[MAIN] Point 4: x={}; y={}".format(self.point_four.x, self.point_four.y))
            rospy.loginfo("[MAIN] Point 6: x={}; y={}".format(self.point_six.x, self.point_six.y))

        else:
            raise Exception("[GoalsList] Could not open yaml file with goals listed!")

    def add_point(self, x, y, orientation, reward, zone):
        goal_point = Goal(y, x, orientation, reward, zone)
        if zone == 'easy':
            self.easy_zone_list.append(goal_point)
        elif zone == 'hard':
            self.hard_zone_list.append(goal_point)

    def remove_easy_zone_point(self, index):
        del self.easy_zone_list[index]

    def remove_hard_zone_point(self, index):
        del self.hard_zone_list[index]

    def navigating_easy_zone(self):
        current_goal_index = 0
        # self.sort_easy_zone_list()
        while len(self.easy_zone_list) > 0:
            status = self.move_base_controller.move_base(self.easy_zone_list[current_goal_index])
            if status is True:
                self.total_reward += self.easy_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                rospy.loginfo("[GoalsList] Remaining goals in easy zone: %s", str(len(self.easy_zone_list) - 1))
                self.remove_easy_zone_point(current_goal_index)
                current_goal_index = 0
                # self.sort_easy_zone_list()
            else:
                if current_goal_index < len(self.easy_zone_list) - 1:
                    current_goal_index += 1
        rospy.loginfo("[GoalsList] All easy zone points visited.")
        rospy.loginfo("[GoalsList] {0}".format(str(len(self.hard_zone_list))))

    def navigating_hard_zone(self):
        current_goal_index = 0
        self.sort_hard_zone_list()
        while len(self.hard_zone_list) > 0:
            status = self.move_base_controller.move_base(self.hard_zone_list[current_goal_index])
            if status is True:
                self.total_reward += self.hard_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                rospy.loginfo("[GoalsList] Remaining goals in hard zone: %s", str(len(self.hard_zone_list) - 1))
                self.remove_hard_zone_point(current_goal_index)
                current_goal_index = 0
                self.sort_hard_zone_list()
            else:
                if current_goal_index < len(self.hard_zone_list) - 1:
                    current_goal_index += 1
        rospy.loginfo("[GoalsList] All hard zone points visited.")
        rospy.loginfo("[GoalsList] {0}".format(str(len(self.hard_zone_list))))

    def get_point_four(self):
        return self.point_four
    
    def get_point_six(self):
        return self.point_six
    
    def sort_easy_zone_list(self):
        self.easy_zone_list.sort(key=self.get_distance_to_current)

    def sort_hard_zone_list(self):
        self.hard_zone_list.sort(key=self.get_distance_to_current)

    def get_distance_to_current(self, goal):
        return math.sqrt((goal.x - self.current_position.x)**2 + (goal.y - self.current_position.y)**2)

    def calculate_list_length(self, list):
        return len(list) - 1