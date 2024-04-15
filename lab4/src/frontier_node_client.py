#!/usr/bin/env python3
from __future__ import annotations
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from path_planner import PathPlanner
from lab4.srv import Cspace
import copy

class FrontierNodeClient:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("frontier_node_client")
        rospy.Subscriber('/map', OccupancyGrid, self.frontier_path_handler)
        #self.frontier_nav_service = rospy.Service('/map', GetMap, self.frontier_path_handler)



    #    self.cspace_pub = rospy.Publisher('/frontier_path/cspace', GridCells, queue_size=10)

        #gets map from gmapping node
        #rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)


    
    #commulative service that takes in a  map(Occumpancy Grid)
    #returns a poseStamped?(a place in the frontier to navigate to)
    def frontier_path_handler(self, map:OccupancyGrid):
        #requestion map from gmapping
        print("in th hnadler")
        #map = self.get_map()
        #do something with map to find a place to go in frontier based on occupancy grid
        
        rospy.wait_for_service('c_space')
        # start = self.start_pose
        # rospy.loginfo(msg)
        # goal = PoseStamped()
        # goal.pose = msg.pose
        # goal.header = msg.header

        try:
            c_space_call = rospy.ServiceProxy('c_space', Cspace)
        #    path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
            print("recievd cspace")
            
            resp1 = c_space_call()
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
    def get_map(self):
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
    

    def edge_detection(self, msg: OccupancyGrid):
        
        pass
    
    
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        map_resolution = mapdata.info.resolution
        world_origin_x = mapdata.info.origin.position.x
        world_origin_y = mapdata.info.origin.position.y

        cell_coordinate_x = int((wp.x - world_origin_x) / map_resolution)
        cell_coordinate_y = int((wp.y - world_origin_y) / map_resolution)

        

        cell_position = (cell_coordinate_x, cell_coordinate_y)

        return cell_position
        
    


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNodeClient().run()
    
