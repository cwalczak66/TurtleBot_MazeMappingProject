#!/usr/bin/env python3
from __future__ import annotations
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import GridCells, OccupancyGrid, Path, OccupancyGridUpdates


class FrontierNode:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("frontier_node")
        # rospy.Subscriber('/map', OccupancyGrid, self.frontier)

        
        self.frontier_nav_service = rospy.Service('map', OccupancyGrid, self.frontier_path_handler)

        #gets map from gmapping node
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdates, queue_size=10)


    
    #commulative service that takes in a  map(Occumpancy Grid)
    #returns a poseStamped?(a place in the frontier to navigate to)
    def frontier_path_handler(self, map:OccupancyGrid):
        #requestion map from gmapping
        map = get_map()
        #do something with map to find a place to go in frontier based on occupancy grid

        
        return
    

    #request map from gampping
    def get_map():
        return
    
    #gets the map from gmapping(topic is /map)
    def update_map():
        return
    
    #update the rviz map by publishing to map_updates
    #will update the rivz and thats it as only sub is rviz
    def update_rivz():
        return