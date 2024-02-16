To reset the move_base costmap in ROS 1 using Python, you can call the clear_costmaps service provided by move_base. Here’s a simple example of how you can do this:

Python
```
import rospy
from std_srvs.srv import Empty

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        resp = clear_costmaps_service()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

# Call the function to clear costmaps
clear_costmaps()
```

AI-generated code. Review and use carefully. More info on FAQ.
This script waits for the clear_costmaps service to become available, then calls the service to clear the costmaps12. Please replace '/move_base/clear_costmaps' with your actual service name if it’s different.

Please note that this will clear all layers of the costmap. If you want to clear a specific layer of the costmap, you might need to disable your layer using dynamic reconfigure1. Be aware that clearing the costmap can sometimes lead to the planner generating a path through obstacles in the static_map3. So, use this with caution.

Let me know if you have any other questions! 😊
