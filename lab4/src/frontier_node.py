
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

        
        self.frontier_cspace = rospy.Service('map', OccupancyGrid, self.calc_cspace)

        #gets map from gmapping node
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdates, queue_size=10)
