from __future__ import annotations
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from path_planner import PathPlanner

def path_planner_client(): 
    
    rospy.wait_for_service('path_planner')

    try:
        path_planner_call = rospy.ServiceProxy('path_planner', PathPlanner)
        

        return path_planner_call
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
