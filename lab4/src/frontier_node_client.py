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
from path_planner_client import PathPlannerClient
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

        self.frontier_centroids_pub = rospy.Publisher('/frontier_centroids', GridCells, queue_size=10)

    #    self.cspace_pub = rospy.Publisher('/frontier_path/cspace', GridCells, queue_size=10)

        #gets map from gmapping node
        #rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)
        #update odom
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.px = 0
        self.py = 0
        self.kp = 0.1
        
        # yaw angle
        self.pth = 0  



    
    #commulative service that takes in a  map(Occumpancy Grid)
    #returns a poseStamped?(a place in the frontier to navigate to)
    def frontier_path_handler(self, map:OccupancyGrid):
        #requestion map from gmapping
        print("in the handler")

        plan = PathPlanner
        padding_cells = plan.calc_cspace2(plan, map, 1)

        self.cspace_pub.publish(plan.makeDisplayMsg(plan, map, padding_cells))

        edge_cell_list = self.edge_detection(map)
        #frontier_c_list = self.frontier_centroid()

        print("Got the edge cells!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111")

        self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, map, edge_cell_list))

        #self.frontier_centroids_pub.publish(plan.makeDisplayMsg(map, frontier_c_list))

        

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

    

    def frontier_centroid(self, list_of_edge_cell_lists: list[list[tuple[int, int]]]) -> list[tuple[int, int]]:

        cell_coordinate_x = 0
        cell_coordinate_y = 0
        # resolution = mapdata.info.resolution
        frontier_centroid_list = []

        for edge_cell_list in list_of_edge_cell_lists:
            num_items_in_sublist = len(edge_cell_list)
            print("number of items: " + str(num_items_in_sublist))
            print(type(edge_cell_list))
        
            for x, y in edge_cell_list:
                cell_coordinate_x += x
                cell_coordinate_y += y
                print("x coordinate: " + str(cell_coordinate_x))
                print("y coordinate: " + str(cell_coordinate_y))  

            cell_coordinate_x /= num_items_in_sublist 
            cell_coordinate_y /= num_items_in_sublist 
        
        frontier_centroid = (cell_coordinate_x, cell_coordinate_y)
        frontier_centroid_list.append(frontier_centroid)
        print(frontier_centroid_list)

        # grid_cell_msg = GridCells()
        # grid_cell_msg.header.stamp = rospy.Time.now()
        # grid_cell_msg.header.frame_id = "map"
        # grid_cell_msg.cell_height = resolution
        # grid_cell_msg.cell_width = resolution
        # grid_cell_msg.cells = frontier_centroid_list

        #self.frontier_centroids_pub.publish(PathPlanner.makeDisplayMsg(mapdata, frontier_centroid_list))
        
        return frontier_centroid_list
        

    
    #works by grapping odom data and then calculating the closest euclidean distance to a frontier and moving towards it
    def move_to_frontier(self, list_of_centroids: list[tuple[int,int]]) -> PoseStamped:
        
        
        rx = self.px
        ry = self.py
        shortest_distance = 100000
        current_tuple = (0,0)
        
        #loop to find closest
        for x,y in list_of_centroids:
            current_dist = PathPlanner.euclidean_distance((rx, ry),(x,y))
            if current_dist < shortest_distance:
                current_tuple = (x,y)
                shortest_distance = current_dist
        
        #not going full way
        going_partway = (current_tuple[0]/2, current_tuple[1]/2)
        #creating pose_stamped
        go_to_pose = PoseStamped()
        go_to_pose.pose.position.x = going_partway[0]
        go_to_pose.pose.position.y = going_partway[1]
        #go_to_pose.pose.orientation
        #moving to point with astar
        PathPlannerClient.path_planner_client(self, go_to_pose)
        
        #return what point robot is going
        return go_to_pose

    
    
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
        
    def update_odometry(self, odom_msg: Odometry):
    
    
        self.px = odom_msg.pose.pose.position.x + 4.8
        self.py = odom_msg.pose.pose.position.y + 4.8
        quat_orig = odom_msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNodeClient().run()
    
