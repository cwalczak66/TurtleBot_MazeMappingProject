#!/usr/bin/env python3

from __future__ import annotations
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from math import sqrt
from priority_queue import PriorityQueue
from visualization_msgs.msg import Marker, MarkerArray
import copy




class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.path_plan_service = rospy.Service('plan_path', GetPlan, self.plan_path_handler)
        self.c_space_service = rospy.Service('c_space', GetMap, self.calc_cspace)

        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells

        self.cspace_pub = rospy.Publisher('/plan_path/cspace', GridCells, queue_size=10)

        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.astar_pub_expandedCells = rospy.Publisher('/plan_path/astar/expandedCells', GridCells, queue_size=10)
        self.astar_pub_frontier = rospy.Publisher('/plan_path/astar/frontier', GridCells, queue_size=10)

        # Publisher for visualizing grid cells visited
        self.cells_visited_astar = rospy.Publisher('/plan_path/cell_visited_astar', GridCells, queue_size=10)

        #publisher for the path message
        self.path_solution = rospy.Publisher('/plan_path/solution_path', Path, queue_size=10)

        #Subscribing to cmd_vel topic to recieve messages about the goal 
    #    self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.plan_path_handler)

        ## Initialize the request counter
        self.request_counter = 0
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

        

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        map_width = mapdata.info.width
        index = p[1]*map_width + p[0]

        return index



    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        ### REQUIRED CREDIT
        initial_point_x = p1[0]
        initial_point_y = p1[1]
        final_point_x = p2[0]
        final_point_y = p2[1]


        euclid_distance = abs(sqrt(pow(final_point_y - initial_point_y, 2 ) + (pow(final_point_x - initial_point_x, 2))**2))

        return euclid_distance
        


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        map_resolution = mapdata.info.resolution
        print(map_resolution)
        world_coordinate_x = (p[0] + 0.5) * map_resolution + mapdata.info.origin.position.x
        world_coordinate_y = (p[1] + 0.5) * map_resolution + mapdata.info.origin.position.y

        

        retval = Point()
        retval.x = world_coordinate_x
        retval.y = world_coordinate_y
        
        return retval
        
        
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
        


        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        
        ### REQUIRED CREDIT
       
        poses_list = []
        previous = None
        for node in path:
            new_pose = PoseStamped()
            new_pose.header.frame_id = "map"
            if previous is not None:
                
                # new_pose.pose.position.x = PathPlanner.grid_to_world(mapdata, node).x - mapdata.info.origin.position.x -0.5
                # new_pose.pose.position.y = PathPlanner.grid_to_world(mapdata, node).y - mapdata.info.origin.position.y -0.5
                new_pose.pose.position.x = node[0]*mapdata.info.resolution
                new_pose.pose.position.y = node[1]*mapdata.info.resolution

                if (node[0] == previous[0] and node[1] > previous[1]): #North
                    new_pose.pose.orientation.z = 0.707
                    new_pose.pose.orientation.w = 0.707
                if (node[0] == previous[0] and node[1] < previous[1]): #South
                    
                    new_pose.pose.orientation.z = -0.707
                    new_pose.pose.orientation.w = 0.707
                if (node[0] > previous[0] and node[1] == previous[1]): #East
                    
                    new_pose.pose.orientation.w = 1.0
                if (node[0] < previous[0] and node[1] == previous[1]): #west
                    new_pose.pose.orientation.z = 1.0

                


                if(node[0] > previous[0] and node[1] > previous[1]): #NE
                    
                    new_pose.pose.orientation.w = 0.924
                    new_pose.pose.orientation.z = 0.383
                if(node[0] < previous[0] and node[1] > previous[1]): #NW
                    new_pose.pose.orientation.w = 0.383
                    new_pose.pose.orientation.z = 0.924
                if(node[0] > previous[0] and node[1] < previous[1]): #SE
                    new_pose.pose.orientation.w = 0.924
                    new_pose.pose.orientation.z = -0.383
                if(node[0] < previous[0] and node[1] < previous[1]): #SW
                    new_pose.pose.orientation.w = 0.383
                    new_pose.pose.orientation.z = -0.924
                    
                   
            else:
                # new_pose.pose.position.x = PathPlanner.grid_to_world(mapdata, node).x - mapdata.info.origin.position.x 
                # new_pose.pose.position.y = PathPlanner.grid_to_world(mapdata, node).y - mapdata.info.origin.position.y
                new_pose.pose.position.x = node[0]*mapdata.info.resolution
                new_pose.pose.position.y = node[1]*mapdata.info.resolution
                new_pose.pose.orientation.w = 1.0

            poses_list.append(new_pose)
            previous = node

            rospy.loginfo(new_pose)

        
        return poses_list
        
    

    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        map_boundary_x = mapdata.info.width # no.of cells
        map_boundary_y = mapdata.info.height # no.of cells
        map_origin_x = int(mapdata.info.origin.position.x) + 5
        map_origin_y = int(mapdata.info.origin.position.y) + 5
        cell_walkable = True
        cell_free = True

        print("origin x = %s",map_origin_x)
        print("origin y = %s",map_origin_y)
        print("boundary= %s", map_boundary_x)

        if(p[0] < map_origin_x or p[0] >= map_origin_x + map_boundary_x or
           p[1] < map_origin_y or p[1] >= map_origin_y + map_boundary_y):
          
          
          return cell_walkable == False
        
        # if p[0] not in range(map_origin_x, map_boundary_x) or p[1] not in range(map_origin_y, map_boundary_y):
        #     print("cell is not walkable")
        #     return cell_walkable == False
        else:
                # cell_index = (p[1] - map_origin_y) * map_boundary_x + (p[0] - map_origin_x)
                # x = mapdata.data[cell_index]
            x = PathPlanner.grid_to_index(mapdata, p)  
            value = mapdata.data[x]  
                  
            if value >= 50 or value == -1:
                cell_free = False
                #print("cell not free")
            
            if cell_free == False:
                cell_walkable = False
                #print("cell is not walkable")
            else:
                cell_free = True
                cell_walkable = True
                #print("cell is walkable")

        #print("cell is walkable")     
        return cell_walkable
        



    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        # neighbours of 4 - neighbouring cells that connect to the face/edge of a square [cell]
        map_boundary_x = mapdata.info.width 
        map_boundary_y = mapdata.info.height
        map_origin_x = mapdata.info.origin.position.x
        map_origin_y = mapdata.info.origin.position.y
        cell_neighbours4 = []

        # if PathPlanner.is_cell_walkable(mapdata, p) == True:
        #     print("cell_walkable")
        cell_edge_neighbour_top = (p[0], p[1]+1)
        if PathPlanner.is_cell_walkable(mapdata, cell_edge_neighbour_top):
            cell_neighbours4.append(cell_edge_neighbour_top)

        cell_edge_neighbour_bottom = (p[0], p[1]-1)
        if PathPlanner.is_cell_walkable(mapdata, cell_edge_neighbour_bottom):
            cell_neighbours4.append(cell_edge_neighbour_bottom)

        cell_edge_neighbour_right = (p[0]+1, p[1])
        if PathPlanner.is_cell_walkable(mapdata, cell_edge_neighbour_right):
            cell_neighbours4.append(cell_edge_neighbour_right)

        cell_edge_neighbour_left = (p[0]-1, p[1])
        if PathPlanner.is_cell_walkable(mapdata, cell_edge_neighbour_left):
            cell_neighbours4.append(cell_edge_neighbour_left)
        
       
        return cell_neighbours4


    
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        map_boundary_x = mapdata.info.width 
        map_boundary_y = mapdata.info.height
        map_origin_x = mapdata.info.origin.position.x
        map_origin_y = mapdata.info.origin.position.y

        cell_neighbours8 = []
        cell_neighbours8 = PathPlanner.neighbors_of_4(mapdata,p)

        cell_topRightcorner = (p[0]+1, p[1]+1)
        if PathPlanner.is_cell_walkable(mapdata, cell_topRightcorner):
            
            cell_neighbours8.append(cell_topRightcorner)

        cell_topLeftcorner = (p[0]-1, p[1]+1)
        if PathPlanner.is_cell_walkable(mapdata, cell_topLeftcorner):
            cell_neighbours8.append(cell_topLeftcorner)

        cell_bottomRightcorner = (p[0]+1, p[1]-1)
        if PathPlanner.is_cell_walkable(mapdata, cell_bottomRightcorner):
            cell_neighbours8.append(cell_bottomRightcorner)

        cell_bottomLeftcorner = (p[0]-1, p[1]-1)
        if PathPlanner.is_cell_walkable(mapdata, cell_bottomLeftcorner):
            cell_neighbours8.append(cell_bottomLeftcorner)

        

        return cell_neighbours8


    
    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/static_map')

        try:  
            get_map = rospy.ServiceProxy('/static_map', GetMap)
            
            return get_map().map
        

        except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        ## Create a GridCells message and publish it
        ## Return the C-space

        map_width = mapdata.info.width
        curr_mapData = copy.deepcopy(mapdata)
        curr_mapData.data = list(curr_mapData.data)
        padded_map_list = []

        new_mapData =  copy.deepcopy(curr_mapData)
        new_mapData.data = list(new_mapData.data)

        for amt_padded in range(0,padding):
            #padded_map_list = []
            for cell_index in range(len(curr_mapData.data)):
                value = curr_mapData.data[cell_index]
                if value > 50: # Identifying any value above 50 in the occupancy grid as obstacle
                    cell_coordinate_y = int(cell_index / map_width)
                    cell_coordinate_x = int(cell_index - (cell_coordinate_y * map_width))
                    cell_coordinate = (cell_coordinate_x, cell_coordinate_y)
                    print(cell_coordinate)
                    print(value)
                    padded_map_list.append(cell_coordinate)
                #    print(padded_map_list)
            #print(padded_map_list)

                    for coordinate in PathPlanner.neighbors_of_8(curr_mapData, cell_coordinate):
                        # new_mapData.data[PathPlanner.grid_to_index(new_mapData, thick)] = 100 # increasing the cell thickness by 100 (1 cell)
                        coordinate_index = self.grid_to_index(curr_mapData, coordinate)
                        if coordinate_index is not None:
                            new_mapData.data[coordinate_index] = 100 
                            padded_map_list.append(coordinate)
            curr_mapData = copy.deepcopy(new_mapData)
            curr_mapData.data = list(new_mapData.data)
        # cspace_mapData = copy.deepcopy(padded_map_list)                    
                
        # mapdata = cspace_mapData
        print(padded_map_list)
        print(mapdata)
        print("Printing cspace")
        self.cspace_pub.publish(self.makeDisplayMsg(curr_mapData,padded_map_list))
        
        ## Return the C-space
        return curr_mapData 
        
    

    def heuristic(self, a: tuple[int,int], b: tuple[int,int]) -> int:
        ax = a[0]
        ay = a[1]
        bx = b[0]
        by = b[1]

        return abs(ax-bx) + abs(ay-by) #Manhattan distance
    
    def cost(self, a: tuple[int,int], b: tuple[int,int]) -> int:

        
        ax = a[0]
        ay = a[1]
        bx = b[0]
        by = b[1]

        return sqrt(pow(ax - bx, 2) + pow(ay - by, 2)) #Euclidean distance

    def reconstruct_path(self, mapdata: OccupancyGrid, came_from: list[tuple[int,int]], start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:

        current = goal
        path = []
        if goal not in came_from: # no path was found
            print("no path found")
            return []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start) # optional
        path.reverse() # optional
        return path
    
    
    
    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        print(mapdata.info.width)
        print(mapdata.info.height)
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current = frontier.get()
            
            if current == goal:
                break
            
            for next in self.neighbors_of_8(mapdata, current):
                new_cost = cost_so_far[current] + self.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current
                    #rospy.sleep(0.01)
                    
                    
                    
        
        self.astar_pub_expandedCells.publish(self.makeDisplayMsg(mapdata, list(came_from)))
        path = self.reconstruct_path(mapdata, came_from, start, goal)
        self.cells_visited_astar.publish(self.makeDisplayMsg(mapdata, path))
        
 
        rospy.loginfo(path)
        return path
    
    
    
    def makeDisplayMsg(self, mapdata: OccupancyGrid,  list_of_nodes: list[tuple[int, int]]):
        
        point_list: Point = []
        resolution = mapdata.info.resolution

        for node in list_of_nodes:
            point_list.append(PathPlanner.grid_to_world(mapdata, node))

        # Set the header
        grid_cell_msg = GridCells()
        grid_cell_msg.header.stamp = rospy.Time.now()
        grid_cell_msg.header.frame_id = "map"
        grid_cell_msg.cell_height = resolution
        grid_cell_msg.cell_width = resolution
        grid_cell_msg.cells = point_list

        return grid_cell_msg

    
    @staticmethod
    def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        rospy.loginfo("Optimizing path")
        rm_node = True
        current = None
        direction = "E"
        optimized_path = copy.deepcopy(path)
        
        for i in range(len(path) - 1):
            current = path[i]
            next = path[i+1]
            new_direction = PathPlanner.check_change_direction(direction, next, current)
            if direction == new_direction:
                print("direction is the same, removing node: " + str(current) + " direction: " + new_direction)
                rospy.loginfo(current)
                optimized_path.remove(current)
            direction = PathPlanner.check_change_direction(direction, next, current)
                
        
        rospy.loginfo(optimized_path)
        return optimized_path
                
          
                
            
        
            
    def check_change_direction(direction, a: list[tuple[int, int]], b: list[tuple[int, int]]):
        
        current_direction = None

        if (a[0] == b[0] and a[1] > b[1]):
            current_direction = "N"
        if (a[0] == b[0] and a[1] < b[1]):
            current_direction = "S"
        if (a[0] > b[0] and a[1] == b[1]):
            current_direction = "E"
        if (a[0] < b[0] and a[1] == b[1]):
            current_direction = "W"

        if (a[0] > b[0] and a[1] > b[1]):
            current_direction = "NE"
        if (a[0] > b[0] and a[1] < b[1]):
            current_direction = "SE"
        if (a[0] < b[0] and a[1] < b[1]):
            current_direction = "SW"
        if (a[0] < b[0] and a[1] > b[1]):
            current_direction = "NW"
        
        print(current_direction)

        return current_direction
   

    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        #original path is in grid, returned is in world
        ### REQUIRED CREDIT
        

        poses_list = self.path_to_poses(mapdata,path)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = poses_list
            
        
        rospy.loginfo("Returning a Path message")
        return(path_msg)





        
    def plan_path_handler(self, msg:PoseStamped):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        #rospy.wait_for_service('map_service')
        print("In Plan_path!")
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        print(mapdata.info.resolution)
        print(mapdata.info.height)
        print(mapdata.info.width)
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*

    #    start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
    #    goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
    #    path  = self.a_star(mapdata, start, goal)

        start = PathPlanner.world_to_grid(cspacedata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(cspacedata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Return a Path message
        


        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        self.path_solution.publish(self.path_to_message(cspacedata, waypoints)) 

        
        return self.path_to_message(cspacedata, waypoints)


    
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
        self.calc_cspace()

        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
