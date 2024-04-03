#!/bin/env python3

from __future__ import annotations
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from path_planner import PathPlanner
from tf.transformations import euler_from_quaternion


class PathPlannerClient:

    def __init__(self):
        # suscribing to 2d nav goal 
        print("starting client")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.path_planner_client)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.start_pose = PoseStamped()


    def path_planner_client(self, msg: PoseStamped): 
    
        rospy.wait_for_service('plan_path')
        start = self.start_pose
        goal = msg

        try:
            path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
            resp = path_planner_call(start, goal, 0)
            return resp
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    
    
    
    def update_odometry(self, msg: Odometry):
        self.start_pose = msg
    
