# Final Tier!

Let'z face the final project

![](https://fbe-gitlab.hs-weingarten.de/mat-iki/amr-mat/raw/master/.img/tier_z.png)

# Final project
The final project is the development of a algorithmn which uses gmapping and move_base to drive to 10 specific points.
The program has the following tasks:
1. The robot moves in the easy zone and collects all goals
2. The robot uses an manual maneuver to drive into the hard zone
3. The robot moves in the hard zone and collects all goals

If you want to learn more about how my code works, you can visit my wiki is available with this [Link](Wiki/_sidebar).


# Gettings Started
## Prerequisites for using my code
1.  An map of the target area so that the robot is able to navigate
2. An list of goals with information about the goals in an yaml file. The one goal should have the following information:
  * The x coordinate as an `float`
  * The y coordinate as an `float`
  * The orientation as an `float`
  * The weighting of the goal  as an `int`
  * The zone of the point. This should indicate if it is the `easy`or `hard` zone

There is already an example list with goals given for the final competition on the 16th February 2024. It is saved in the repository under `/fs-213804_tier4/final_project/config/final_goals.yaml`.

The easiest way of making the code runnable with a new robot arena is to use this list and replace the x and y variables with the new variables.


## Steps for the launch of the robot
1. Change the rosmaster ip on your device to the ip addess of the robot and source the ~/.bashrc
2. Connect to the robot via ssh e.g. for the robot with the ip 192.168.178.5
    ```
    ssh 192.168.178.5
    ```
3. Start the robot with this command:
    ```
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
4. The next step is the start of navigation via this command:
    ```
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<absolute_path_to_map>.yaml
    ```
    
    In my case was it the following command with the link to my file location in the repository:
    ```
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/fabian/catkin_ws/src/fs-213804_tier4/final_project/final_map.yaml
    ```
5.  The next step is the localization of the robot on the map. This can be done in two different ways:
    * The first way is of estimating the current position and selecting it on the map
    * The second way is to use the following command:
      ```
      roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
      ```
6.  The last step for using this code is the launch of the robot by executing this command:
    ```
    roslaunch final_project start.launch
    ```

# Problems and Solutions
### The bringup command does not work.
  * Is the correct rosmaster ip in the `~/.bashrc` used and is it sourced? If not, correct the rosmaster ip on your device and source it by executing `source ~/.bashrc` 
### The start of the navigation does not work because the map could not be found
  * There are a few reasons for this which should be checked
    * Does an map with the used name exist?
    * Is the given path in the command an absolute path?
### The robot does not hit the goals correctly
  * This could be because the robot is not correctly localized. Check **step 5 of Getting Started** to correct this.
  * If the robot is correctly localized, it could be that the goal positions are inaccurate. In this case correct the goal positions by remapping the goals. Check **prerequisite no. 2** for this.

# Author
* Fabian Schotte, 35604
