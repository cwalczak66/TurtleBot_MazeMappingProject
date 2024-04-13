
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

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # self.grid_to_index()
        # self.euclidean_distance()
        # self.grid_to_world()
        # self.is_cell_walkable()
        # self.neighbors_of_4()
        # self.neighbors_of_8()
        # self.a_star()

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNode.run()
