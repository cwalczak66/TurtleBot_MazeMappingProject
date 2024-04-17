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
from tf.transformations import euler_from_quaternion

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

        shape_list = self.edge_detection2(map)
        edges = []

        for shape in shape_list:
            for e_cell in shape:
                edges.append(e_cell)

        
        self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, map, edges))
            
            

        

        print("Got the edge cells!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111")



       


       
    

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
    
    def find_shape(self, mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        shape = []
        shape.append(p)
        notDone = True
        plan = PathPlanner
        queue = []     #Initialize a queue
        visited = [] # List for visited nodes.
        visited.append(p)
        queue.append(p)
        while queue:


            m = queue.pop(0)
            shape.append(m)
            self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, mapdata, shape))
            rospy.sleep(0.07)
            
            
            N_list = PathPlanner.neighbors(mapdata, m)
            for e_cell in N_list:
                index = PathPlanner.grid_to_index(mapdata, e_cell)
                value = mapdata.data[index]
                if (self.is_edge(mapdata, e_cell)) and PathPlanner.is_cell_walkable(mapdata, e_cell) and e_cell not in visited:
                    visited.append(e_cell)
                    queue.append(e_cell)

        print("done, returning shape")
        return shape
                    

        
    def is_edge(self, mapdata: OccupancyGrid, cell)-> bool:
        isEdge = False
        n_list = PathPlanner.neighbors(mapdata, cell)
        for n_cell in n_list:
            index = PathPlanner.grid_to_index(mapdata, n_cell) 
            value = mapdata.data[index]
            if value == -1 :
                isEdge = True
        return isEdge
    

    def edge_detection2(self, mapdata: OccupancyGrid)-> list[list[tuple[int, int]]]:
        cell_list = []
        shapes = []
        
        map_width = mapdata.info.width
        for cell_index in range(len(mapdata.data)):
                
                
            cell_coordinate_y = int(cell_index / map_width)
            cell_coordinate_x = int(cell_index - (cell_coordinate_y * map_width))
            cell_coordinate = (cell_coordinate_x, cell_coordinate_y)
            if PathPlanner.is_cell_walkable(mapdata, cell_coordinate) and self.is_edge(mapdata, cell_coordinate):
                cell_list.append(cell_coordinate)

        while cell_list:
            shape = self.find_shape(mapdata, cell_list[0])
            shapes.append(shape)
            for c in shape:
                if c in cell_list:
                    cell_list.remove(c)


            
            
        return shapes

        


    

    def frontier_centroid(self, list_of_cells: list[tuple[int, int]]) -> tuple[int, int]:

        cell_coordinate_x = 0
        cell_coordinate_y = 0

        num = len(list_of_cells)
        for x, y in list_of_cells:
            cell_coordinate_x += x
            cell_coordinate_y += y
        
        cell_coordinate_x = cell_coordinate_x / num
        cell_coordinate_y = cell_coordinate_y / num

        return (cell_coordinate_x, cell_coordinate_y)
    
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
    
