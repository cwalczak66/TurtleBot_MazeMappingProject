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
        rospy.init_node("path_planner_client")
        print("starting client")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.path_planner_client)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.start_pose = PoseStamped()


    def path_planner_client(self, msg: PoseStamped): 
    
        rospy.wait_for_service('plan_path')
        start = self.start_pose
        rospy.loginfo(msg)
        goal = PoseStamped()
        goal.pose = msg.pose
        goal.header = msg.header

        try:
            path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
            resp = path_planner_call(start, goal, 0)
            return resp
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    
    
    
    def update_odometry(self, odom_msg: Odometry):

        # px = msg.pose.pose.position.x
        # py = msg.pose.pose.position.y
        # quat_orig = msg.pose.pose.orientation
        # quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        # (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        # pth = yaw

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header = odom_msg.header
        pose_stamped_msg.pose = odom_msg.pose.pose
        self.start_pose = pose_stamped_msg
        

    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
       
        rospy.spin()


        
if __name__ == '__main__':
    PathPlannerClient().run()

    
