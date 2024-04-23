#!/usr/bin/env python3
from __future__ import annotations
from queue import Empty
from lab2.src.lab2 import Lab2
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from path_planner import PathPlanner
#from lab4.srv import Cspace
from path_planner import PathPlanner
from path_planner_client import PathPlannerClient
import copy
from tf.transformations import euler_from_quaternion
from math import pi,sum
import numpy as np

class FrontierNodeClient:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("frontier_node_client")
        rospy.Subscriber('/map', OccupancyGrid, self.frontier_path_handler)
        self.go_to_pub = rospy.Publisher('move_base_simple/goal' , PoseStamped, queue_size=10)


        self.cspace_pub = rospy.Publisher('/plan_path/cspace', GridCells, queue_size=10)
        self.edge_cells_pub = rospy.Publisher('/edge_cells', GridCells, queue_size=10)
        self.gotroid_pub = rospy.Publisher('/gotroid', GridCells , queue_size=10 )
        #self.frontier_nav_service = rospy.Service('/map', GetMap, self.frontier_path_handler)

        self.frontier_centroids_pub = rospy.Publisher('/frontier_centroids', GridCells, queue_size=10)

    #    self.cspace_pub = rospy.Publisher('/frontier_path/cspace', GridCells, queue_size=10)

        #gets map from gmapping node
        #rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #subscriber for amcl
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped , self.update_covariance)
        
        #WHEN THE AMCL WORKS WE GIVE IT ANOTHER NAV GOAL AFTER LOCALIZATION
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.amcl_move)





        #update map in rviz
        self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)
        #update odom
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.px = 0
        self.py = 0 
        self.kp = 0.1

        self.start_pose = PoseStamped()
        # yaw angle
        self.pth = 0  
        self.first_bool = True
        self.starting_position = ()
 
        #self.going_partway = 0
        self.going_centroid = []
        self.amclx = 0
        self.amcly = 0
        self.amcl_theta = 0
        self.amclcovariance = 0
        self.amclox = 0
        self.amcloy = 0
        self.amcloz = 0
        self.amclow = 0 


    #Turns the robot once AMCL is running to localize the bot in the map
    #Poststamped msg is the goal pose provided in rviz
    
    # def localization(self):
    #     rospy.loginfo("Requesting localization from AMCL")
    #     localization_request = rospy.ServiceProxy('/global_localization', Empty)
        
    #     return localization_request
    
    def amcl_move(self, msg:PoseStamped):

        number_of_turns = 0 # Start at 0 turns done
        #("rosservice call /global_localization") # Scatter a bunch of potential robot positions around the map for amcl to sort through
        #rospy.loginfo("Requesting localization from AMCL")
        #localization_request = rospy.ServiceProxy('/global_localization', Empty)

        # Do a minimum number of turns and then keep turning until we are very confident about our location or we've turned for too long
        while (number_of_turns < 5 or self.covariance > 0.1) and number_of_turns < 10:
            Lab2.rotate(pi / 2)
            number_of_turns += 1
            rospy.sleep(.75)
        
        # Moving the robot to the goal once localized
        rospy.loginfo("Localized. Driving to goal")
        self.go_to_pub.publish(msg)
        #PathPlannerClient.path_planner_client(self, msg)
    
    #Update the covariance when receiving a message from amcl_pose
    #Covariance is the sum of the diagonal elements of the covariance matrix
    def update_covariance(self, msg: PoseWithCovarianceStamped) -> None:
     
        covariance_matrix = np.reshape(msg.pose.covariance, (6,6)) # 36 long, 6 x 6 covariance matrix
        self.covariance = np.sum(np.diag(covariance_matrix)) # Get sum of diagonal elements of covariance matrix
        
        self.amclx = msg.pose.pose.position.x
        self.amcly = msg.pose.pose.position.y
        self.amcl_theta = msg.pose.pose.orientation.w
    
    

    #commulative service that takes in a  map(Occumpancy Grid)
    #returns a poseStamped(a place in the frontier to navigate to)
    def frontier_path_handler(self, mapdata:OccupancyGrid):
        #requestion map from gmapping
        print("in the handler")
        if self.first_bool == True:
            self.starting_position = (self.px, self.py)
            self.first_bool = False

      

        plan = PathPlanner
        padding_cells = plan.calc_cspace2(plan, mapdata, 1)

        shape_list = self.edge_detection2(mapdata)
        edges = []
        centroids = []

        for shape in shape_list:
            for e_cell in shape:
                edges.append(e_cell)

        centroids = self.frontier_centroid(shape_list)

        for padded_cell in padding_cells:
            for c in centroids:
                if c == padded_cell:
                    centroids.remove(c)
                


        print("LIST SORTED ITS TIME TO MOVE!")
        self.move_to_frontier(centroids)
        #LETS HOME THIS WORK
        print("FINISHED MOVE")
        # rospy.sleep(10)



        self.cspace_pub.publish(plan.makeDisplayMsg(plan, mapdata, padding_cells))

        #self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, mapdata, edges))
        self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, mapdata, centroids))

        self.gotroid_pub.publish(plan.makeDisplayMsg(plan, mapdata, self.going_centroid))
        
            
        print("Got the edge cells!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111")
    
    def return_to_start(self):
        starting_pose = PoseStamped()
        starting_pose.pose.position.x = self.starting_position[0]
        starting_pose.pose.position.y = self.starting_position[1]
        starting_pose.header.frame_id = "map"
        starting_pose.header.stamp = rospy.Time.now()
        PathPlannerClient.path_planner_client(self, starting_pose)


    #request map from gampping
    # def get_map(self):
    #     rospy.loginfo("Requesting the map")
    #     rospy.wait_for_service('/map')

    #     try:  
    #         get_map = rospy.ServiceProxy('/map', GetMap)
            
    #         return get_map().map
        

    #     except rospy.ServiceException as e:
    #      print("Service call failed: %s"%e)
    
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
            # self.edge_cells_pub.publish(plan.makeDisplayMsg(plan, mapdata, shape))
            # rospy.sleep(0.07)
            
            
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

        


    

    def frontier_centroid(self, list_of_edge_cell_lists: list[list[tuple[int, int]]]) -> list[tuple[int, int]]:

        
        # resolution = mapdata.info.resolution
        frontier_centroid_list = []

        for edge_cell_list in list_of_edge_cell_lists:
            num_items_in_sublist = len(edge_cell_list)
            print("number of items: " + str(num_items_in_sublist))
            print(type(edge_cell_list))

            cell_coordinate_x = 0
            cell_coordinate_y = 0
        
            for x, y in edge_cell_list:
                cell_coordinate_x += x
                cell_coordinate_y += y
                print("x coordinate: " + str(cell_coordinate_x))
                print("y coordinate: " + str(cell_coordinate_y))  

            cell_coordinate_x /= num_items_in_sublist 
            cell_coordinate_y /= num_items_in_sublist 
        
            frontier_centroid = (int(cell_coordinate_x), int(cell_coordinate_y))
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
        
        

        shortest_distance = 100000
        current_tuple = (0,0)
        
        #loop to find closest
        print("finding closest")
        for x,y in list_of_centroids:
            current_dist = PathPlanner.euclidean_distance((self.px, self.py),(x,y))
            if current_dist < shortest_distance:
                current_tuple = (x,y)
                shortest_distance = current_dist
        
        #not going full way
        going_partway = (current_tuple[0]/2, current_tuple[1]/2)
        #creating pose_stamped
        go_to_pose = PoseStamped()
        go_to_pose.pose.position.x = going_partway[0]
        go_to_pose.pose.position.y = going_partway[1]
        go_to_pose.header.frame_id = "map"
        go_to_pose.header.stamp = rospy.Time.now()
        
        #go_to_pose.pose.orientation
        #moving to point with astar
        print("FOUND POINT TIME TO ASTAR")
        print(current_tuple[0], current_tuple[1])
        print(going_partway[0], going_partway[1])
        #suppose to move but doesnt :()
        #PathPlannerClient.path_planner_client(self, go_to_pose)

        #global value just for rviz and testing
        self.going_centroid.append(going_partway)

        self.go_to_pub.publish(go_to_pose)
        
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
        #NEED TO UPDATE THE START


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNodeClient().run()
    
