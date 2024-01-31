#! /usr/bin/env python3
import rospy
import yaml
import move_base_controller


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

            self.sort_points()

        else:
            raise Exception("[GoalsList] Could not open yaml file with goals listed!")

    def add_point(self, x, y, orientation, reward, zone):
        goal_point = Goal(x, y, orientation, reward, zone)
        if zone == 'easy':
            self.easy_zone_list.append(goal_point)
        elif zone == 'hard':
            self.hard_zone_list.append(goal_point)

    def sort_points(self):
        try:
            self.easy_zone_list.sort(key=lambda point: point.reward, reverse=True)
            self.hard_zone_list.sort(key=lambda point: point.reward, reverse=True)
        except (TypeError, AttributeError) as e:
            rospy.logerr(f"[GoalsList] An error occurred while sorting points: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"[GoalsList] An unexpected error occurred while sorting points: {e}")
            raise
        else:
            self.combined_list = self.easy_zone_list + self.hard_zone_list

    def print_goals(self):
        for goal in self.combined_list:
            rospy.loginfo("P{0} [GoalsList] x: {1}, y: {2}, reward: {3}, zone: {4}"
                          .format(self.combined_list.index(goal) + 1, goal.x, goal.y, goal.reward, goal.zone))

    def remove_easy_zone_point(self, index):
        del self.easy_zone_list[index]

    def remove_hard_zone_point(self, index):
        del self.hard_zone_list[index]

    def navigating_easy_zone(self):
        current_goal_index = 0
        while len(self.easy_zone_list) > 0:
            status = self.move_base_controller.move_base(self.easy_zone_list[current_goal_index])
            if status is True:
                self.total_reward += self.easy_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                self.remove_easy_zone_point(current_goal_index)
                current_goal_index = 0
            else:
                if current_goal_index < len(self.easy_zone_list) -1:
                    current_goal_index += 1
        rospy.loginfo("[GoalsList] All easy zone points visited.")
        rospy.loginfo("[GoalsList] {0}".format(len(self.easy_zone_list)))

    def navigating_hard_zone(self):
        current_goal_index = 0
        while len(self.hard_zone_list) > 0:
            status = self.move_base_controller.move_base(self.hard_zone_list[current_goal_index])
            if status is True:
                self.total_reward += self.hard_zone_list[current_goal_index].reward
                rospy.loginfo("[GoalsList] Total reward: {0}".format(str(self.total_reward)))
                self.remove_hard_zone_point(current_goal_index)
                current_goal_index = 0
            else:
                if current_goal_index < len(self.easy_zone_list) -1:
                    current_goal_index += 1     
        rospy.loginfo("[GoalsList] All easy zone points visited.")
        rospy.loginfo("[GoalsList] {0}".format(len(self.hard_zone_list)))