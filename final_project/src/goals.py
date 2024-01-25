#! /usr/bin/env python3
import rospy
import yaml


class Goal:
    def __init__(self, x, y, orientation, score):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.score = score


class GoalsList:
    def __init__(self):
        config_file_name = rospy.get_param("~goals_config_file", "goals.yaml")
        rospy.loginfo("Configuration file name: " + str(config_file_name))
        path_to_open = config_file_name
        try:
            ymlfile = open(path_to_open, "r")
            self.configFile = yaml.load(ymlfile, yaml.SafeLoader)
        except Exception as _:
            rospy.logerr("Could not open file " + str(path_to_open) + ".")
            rospy.logerr("Exiting")
        self.easy_zone_list = []
        self.hard_zone_list = []
        self.combined_list = []

    def read_goals(self):
        try:
            goals_yaml = self.configFile["goals"]
        except Exception as _:
            goals_yaml = None
            raise Exception("No tag 'goals' in yaml file found!")

        if goals_yaml is not None:
            # Iterate through goals
            for goal_key, goal_value in sorted(goals_yaml.items()):
                try:
                    rospy.loginfo(goal_value)
                    self.add_point(goal_value['x'], goal_value['y'], goal_value['orientation'], goal_value['score'],
                                   goal_value['zone'])

                except Exception as _:
                    rospy.logerr(
                        """Could not add this point. Skipping it and continuing
                             with the next point"""
                    )

            self.sort_points()

        else:
            raise Exception("Could not open yaml file with goals listed!")

    def add_point(self, x, y, orientation, score, zone):
        goal_point = Goal(x, y, orientation, score)
        if zone == 'easy':
            self.easy_zone_list.append(goal_point)
        elif zone == 'hard':
            self.hard_zone_list.append(goal_point)

    def sort_points(self):
        try:
            self.easy_zone_list.sort(key=lambda point: point.score, reverse=True)
            self.hard_zone_list.sort(key=lambda point: point.score, reverse=True)
        except (TypeError, AttributeError) as e:
            rospy.logerr(f"An error occurred while sorting points: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred while sorting points: {e}")
            raise
        else:
            self.combined_list = self.easy_zone_list + self.hard_zone_list
