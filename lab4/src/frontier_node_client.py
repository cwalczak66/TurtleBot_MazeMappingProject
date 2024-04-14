#!/usr/bin/env python3
from __future__ import annotations
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from path_planner import PathPlanner
import copy

class FrontierNodeClient:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("frontier_node_client")
        # rospy.Subscriber('/map', OccupancyGrid, self.frontier)

        
        self.frontier_nav_service = rospy.Service('map', GetMap, self.frontier_path_handler)

    #    self.cspace_pub = rospy.Publisher('/frontier_path/cspace', GridCells, queue_size=10)

        #gets map from gmapping node
        #rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)


    
    #commulative service that takes in a  map(Occumpancy Grid)
    #returns a poseStamped?(a place in the frontier to navigate to)
    def frontier_path_handler(self, map:OccupancyGrid):
        #requestion map from gmapping
        map = self.get_map()
        #do something with map to find a place to go in frontier based on occupancy grid
        
        rospy.wait_for_service('c_space')
        # start = self.start_pose
        # rospy.loginfo(msg)
        # goal = PoseStamped()
        # goal.pose = msg.pose
        # goal.header = msg.header

        try:
            c_space_call = rospy.ServiceProxy('c_space', GetMap)
        #    path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
            resp1 = c_space_call(map)
            # resp2 = path_planner_call(start, goal, 0)
            # rospy.loginfo(resp.plan.poses)
            # step = 1
            # for waypoint in resp.plan.poses:

                
            #     print("STEP "+ str(step))
            #     print("")
            #     self.go_to(waypoint)
            #     step = step + 1
                
            
            #     print("FINISHED GO_TO")

            return resp1

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    

    #request map from gampping
    def get_map():
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/map')

        try:  
            get_map = rospy.ServiceProxy('/map', GetMap)
            
            return get_map().map
        

        except rospy.ServiceException as e:
         print("Service call failed: %s"%e)
    
    #gets the map from gmapping(topic is /map)
    def update_map():
        return
    
    #update the rviz map by publishing to map_updates
    #will update the rivz and thats it as only sub is rviz
    def update_rivz():
        return
    


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNodeClient().run()
    
