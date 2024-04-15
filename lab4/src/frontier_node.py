#!/bin/env python3

from __future__ import annotations
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from frontier_node_client import FrontierNodeClient


class FrontierNode:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("frontier_node")
        # rospy.Subscriber('/map', OccupancyGrid, self.frontier)

        #gets map from gmapping node
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)


    def update_map(self, msg: OccupancyGrid):
        print("found map")
        return
    
    def detect_edges(self, mapdata: OccupancyGrid):
        unexplored = []
        
            
            
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
       
        rospy.spin()
    
# use this for frontier calc

if __name__ == '__main__':
    FrontierNode().run()
