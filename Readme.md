# Final Tier!

Let'z face the final project

![](https://fbe-gitlab.hs-weingarten.de/mat-iki/amr-mat/raw/master/.img/tier_z.png)

# Final project
The final project is the development of a algorithmn which uses gmapping and move_base to drive to 10 specific points.
My Wiki is available under this [Link](Wiki/_sidebar).

# Gettings Started
1. The map of the area should be created
2. The code needs the goal poinst which will be stored in the *final_goals.yaml* in the *config* folder.
3. Start the code my typing the following command in the terminal:
  ```
  roslaunch final_project start.launch
  ```

# Problems and Solutions
* **I creared multiple goals.yaml files for the receiving of the points and the code could not find the additional files.**
  * The problem was the missing tag in the node tag
  * after adding the required tags, the code could access the yaml files
* **The action client can not send the goals to the action server**
  * 


# Authors
* Fabian Schotte, 35604
