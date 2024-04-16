#!/usr/bin/env python3
from __future__ import annotations
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from path_planner import PathPlanner
#from lab4.srv import Cspace
from path_planner import PathPlanner
import copy

class FrontierNodeClient:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("frontier_node_client")
        rospy.Subscriber('/map', OccupancyGrid, self.frontier_path_handler)
        self.cspace_pub = rospy.Publisher('/plan_path/cspace', GridCells, queue_size=10)
        self.edge_cells_pub = rospy.Publisher('edge_cells', GridCells, queue_size=10)
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
        print("in the handler")

        plan = PathPlanner
        padding_cells = plan.calc_cspace2(plan, map, 1)

        self.cspace_pub.publish(plan.makeDisplayMsg(plan, map, padding_cells))

        edge_cell_list = self.edge_detection(map)

        print("Got the edge cells!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111")

        self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, map, edge_cell_list))


       


       
    

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
    

    def edge_detection(self, mapdata: OccupancyGrid) -> list[tuple[int, int]]:
        cell_list = []
        plan = PathPlanner
        edge_cell_list = []
        map_width = mapdata.info.width
        
        for cell_index in range(len(mapdata.data)):
                
                
                cell_coordinate_y = int(cell_index / map_width)
                cell_coordinate_x = int(cell_index - (cell_coordinate_y * map_width))
                cell_coordinate = (cell_coordinate_x, cell_coordinate_y)

                if plan.is_cell_walkable(mapdata, cell_coordinate):

                    cell_list.append(cell_coordinate)
                

            #     for coordinate in PathPlanner.neighbors_of_8(curr_mapData, cell_coordinate):
            #         # new_mapData.data[PathPlanner.grid_to_index(new_mapData, thick)] = 100 # increasing the cell thickness by 100 (1 cell)
            #         coordinate_index = self.grid_to_index(curr_mapData, coordinate)
            #         if coordinate_index is not None:
            #             new_mapData.data[coordinate_index] = 100 
            #             padded_map_list.append(coordinate)
            # curr_mapData = copy.deepcopy(new_mapData)
            # curr_mapData.data = list(new_mapData.data)

        # list_of_edge_cells = []
        # map_width = mapdata.info.width

        

        
        for cell in cell_list:
      
           
            neighbor_cells = PathPlanner.neighbors(mapdata, cell)
            
            for n_cell in neighbor_cells:
                index = PathPlanner.grid_to_index(mapdata, n_cell)
                 
                value = mapdata.data[index]
                
                if value == -1:
                    edge_cell_list.append(cell)

        print(edge_cell_list)
        return edge_cell_list

    
    
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
    
