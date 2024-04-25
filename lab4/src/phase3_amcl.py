#!/usr/bin/env python3
from __future__ import annotations
from queue import Empty
#from statistics import covariance

from attr import s
#from lab2.src.lab2 import Lab2
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, TransformStamped, Twist, Vector3, PointStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from path_planner import PathPlanner
#from lab4.srv import Cspace
from path_planner import PathPlanner
from path_planner_client import PathPlannerClient
import copy
from tf.transformations import euler_from_quaternion
import tf 
from tf import TransformListener
from std_msgs.msg import Bool
from math import pi
import numpy as np 
from std_srvs.srv import Empty as ros_empty
from geometry_msgs.msg import Quaternion
from math import sqrt , pi, atan2, sin, cos 


class FrontierNodeClient:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("phase3_amcl")
        #rospy.Subscriber('/map', OccupancyGrid, self.frontier_path_handler) 
        #comment out the current forntier_path_handler
        self.go_to_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.dest_pub = rospy.Publisher('/dest', GridCells, queue_size=10)
        # self.cspace_pub = rospy.Publisher('/plan_path/cspace', GridCells, queue_size=10)
        # self.edge_cells_pub = rospy.Publisher('/edge_cells', GridCells, queue_size=10)
        # self.gotroid_pub = rospy.Publisher('/gotroid', GridCells , queue_size=10 )


        #self.frontier_nav_service = rospy.Service('/map', GetMap, self.frontier_path_handler)

        self.frontier_centroids_pub = rospy.Publisher('/frontier_centroids', GridCells, queue_size=10)

    #    self.cspace_pub = rospy.Publisher('/frontier_path/cspace', GridCells, queue_size=10)

        #gets map from gmapping node
        #rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        #subscriber for amcl
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped , self.update_covariance)
        
        #WHEN THE AMCL WORKS WE GIVE IT ANOTHER NAV GOAL AFTER LOCALIZATION


        #initing amcl move
        rospy.Subscriber('clicked_point' , PointStamped , self.amcl_localize)

        rospy.Subscriber('bool_topic', Bool, self.wait_for_waypoint)


        # rospy.Subscriber('/' , PoseStamped, self.drive_home)
        # rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.drive_home)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.drive_home)

       
        


        # #update map in rviz
        # self.update_rivz = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)

        #update odom NEW
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('bool_topic', Bool, self.wait_for_waypoint)
        #rospy.Subscriber("/map", OccupancyGrid, self.get_map)


        self.astar_path_pub = rospy.Publisher('/path_home', GridCells, queue_size=10)

        self.listener = tf.TransformListener()

        self.px = 0
        self.py = 0 
        self.kp = 0.1

        self.start_pose = PoseStamped()
        # yaw angle
        self.pth = 0  
        self.first_bool = True
        self.starting_position = (0, 0)
 
        #self.going_partway = 0
        self.going_centroid = []
        self.amclx = 0
        self.amcly = 0
        self.amcl_theta = 0
        self.covariance = 0
        self.amclox = 0
        self.amcloy = 0
        self.amcloz = 0
        self.amclow = 0 

       


    def request_map(self) -> OccupancyGrid:
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
    
    def print_point(self, msg: PoseStamped):
        
        mapdata = self.request_map()
        print("Robot end pose: " + str((msg.pose.position.x, msg.pose.position.y)) + " in grid: " + str(self.world_to_grid(mapdata, msg.pose.position)))
        p = Point()
        p.x = self.px
        p.y = self.py
        print("Robot current pose: " + str((self.px, self.py)) + " in grid: " + str(self.world_to_grid(mapdata, p)))
        

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        
        msg = Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed))
        ### Publish the message
        self.cmd_pub.publish(msg)
    
    

    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        ang_tol = 0.09
        rospy.wait_for_message("/odom", Odometry) #wait for angle update before execution
        rate = rospy.Rate(10) # Publish rate of 10Hz

        while not rospy.is_shutdown():
            angle_difference = (angle - self.pth)% (2* pi)
            print(f'target angle: {angle * (180/pi)} current angle: {self.pth * (180/pi)} angle difference: {angle_difference * (180/pi)}')
            # Normalizing angle difference to range btw pi and -pi
            #angle_difference = atan2(sin(angle_difference), cos(angle_difference))
            #print(angle_difference)
            while angle_difference > pi:
                angle_difference = angle_difference - 2 * pi
            while angle_difference < -pi:
                angle_difference = angle_difference + 2*pi
            rate.sleep()
            
            if abs(angle_difference) <= ang_tol:
                self.send_speed(0.0,0.0)
                rospy.sleep(0.5)
                self.send_speed(0,0)
                print("reached goal!")
                break
            else:
                # Normalizing angle difference to range btw pi and -pi
                if angle_difference > 0:    
                    self.send_speed(0, aspeed) #clockwise
                else:
                    self.send_speed(0, -aspeed) #anticlockwise
                rospy.sleep(0.5)
        
        self.send_speed(0,0)
        print("robot should stop now")
        self.send_speed(0.0,0.0)

    #Turns the robot once AMCL is running to localize the bot in the map
    #Poststamped msg is the goal pose provided in rviz
    
    # def localization(self):
    #     rospy.loginfo("Requesting localization from AMCL")
    #     localization_request = rospy.ServiceProxy('/global_localization', Empty)
        
    #     return localization_request
    
    def amcl_localize(self, msg:PointStamped):

        number_of_turns = 0 # Start at 0 turns done
        #("rosservice call /global_localization") # Scatter a bunch of potential robot positions around the map for amcl to sort through

        # Do a minimum number of turns and then keep turning until we are very confident about our location or we've turned for too long\
        #WANT COVARIANCE TO BE SMALL

        rospy.wait_for_service('global_localization')
        loc = rospy.ServiceProxy('global_localization', ros_empty)
        loc()
        print("")
        print("default cov: " + str(self.covariance))
        #while (number_of_turns < 10):

        print("covariance value" + str(self.covariance))
        self.rotate(pi, 0.1)
        self.rotate(0.0, 0.1)
        number_of_turns += 1
        rospy.sleep(.75)
            
        
        # Moving the robot to the goal once localized
        rospy.loginfo("Localized. Waiting for destination")
        #rospy.wait_for_message('final_goal', PoseStamped)
        # rospy.wait_for_message(PoseStamped)
        # go_to_msg = PoseStamped()
        # go_to_msg.header.frame_id = "map"
        # go_to_msg.header.stamp = rospy.Time.now()
        # go_to_msg.pose = msg.pose.pose

        # mapdata = self.request_map()
        
        # waypoints = self.get_astar_path(mapdata, go_to_msg)
      

        # for waypoint in waypoints:
        #     self.go_to_pub.publish(waypoint)
        #     print("waiting")
        #     rospy.wait_for_message('bool_topic', Bool)



        # self.go_to_pub.publish(go_to_msg)
        #PathPlannerClient.path_planner_client(self, msg)
    
    #Update the covariance when receiving a message from amcl_pose
    #Covariance is the sum of the diagonal elements of the covariance matrix

    def wait_for_waypoint(self, msg: Bool):
        pass

    def drive_home(self, goal: PoseStamped):
        
        print("in drive")
        print(self.px)
        print(self.py)


        #rospy.wait_for_message(PoseStamped)

        # end = PoseStamped()
        # end.header.frame_id = "map"
        # end.header.stamp = rospy.Time.now()
        # end.pose = msg.pose.pose

        

        mapdata = self.request_map()
        # start = PoseStamped()
        # start.pose.position = Point(self.px, self.py, 0)
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pth)
        # start.pose.orientation = quaternion
        # start.header.frame_id = "map"

        #print("Robot start pose: " + str(self.world_to_grid(mapdata, start.pose.position)))
        print("Robot end pose: " + str(self.world_to_grid(mapdata, goal.pose.position)))
        goal_grid = self.world_to_grid(mapdata, goal.pose.position)
        goal.pose.position.x = goal_grid[0]
        goal.pose.position.y = goal_grid[1]


        waypoints = self.get_astar_path(mapdata, goal)
      

        for waypoint in waypoints:
            #self.go_to_pub.publish(waypoint)
            self.go_to(waypoint)
            print("waiting")
            #rospy.wait_for_message('bool_topic', Bool)

        print("DONE WITH EVERYTHINGS")

        

    # def get_astar_path(self, mapdata: OccupancyGrid, goal: PoseStamped):
    #     plan = PathPlanner
    #     print("calling on server")

    #     start = PoseStamped()
    #     wp = Point()
    #     wp.x = self.px
    #     wp.y = self.py
    #     grid_robot = self.world_to_grid(mapdata, wp)
        
    #     start.pose.position.x = grid_robot[0]
    #     start.pose.position.y = grid_robot[1]
    #     start.header.frame_id = "map"
    #     start.header.stamp = rospy.Time.now()
    #     try:
    #         amcl_path = rospy.ServiceProxy('amcl_srv', GetPlan)
    #         resp = amcl_path(start, goal, 0)
    #         if not resp.plan.poses:
              
    #             return None
    #         else:
    #             #trying something
    #             return resp.poses


    #         #return resp.plan.poses

    #     except rospy.ServiceException as e:
    #         print("Service call has failed: %s"%e)
    #         return None


    def update_covariance(self, msg: PoseWithCovarianceStamped) -> None:
     
        covariance_matrix = np.reshape(msg.pose.covariance, (6,6)) # 36 long, 6 x 6 covariance matrix
        self.covariance = np.sum(np.diag(covariance_matrix)) # Get sum of diagonal elements of covariance matrix
        
        self.amclx = msg.pose.pose.position.x
        self.amcly = msg.pose.pose.position.y
        self.amcl_theta = msg.pose.pose.orientation.w

    def wait_for_waypoint(self, msg: Bool):
        pass

    def get_astar_path(self, mapdata: OccupancyGrid, goal: PoseStamped):
        plan = PathPlanner
        print("calling on server")

        
        start = PoseStamped()
        wp = Point()
        wp.x = self.px
        wp.y = self.py
        grid_robot = self.world_to_grid(mapdata, wp)
        
        start.pose.position.x = grid_robot[0]
        start.pose.position.y = grid_robot[1]
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time.now()

        try:
            path_planner_call = rospy.ServiceProxy('amcl_srv', GetPlan)
            resp = path_planner_call(start, goal, 0)
            #rospy.loginfo(resp.plan.poses)

            # list_of_grid_cells = []

            # for pose in resp.plan.poses:
            #     list_of_grid_cells.append(self.world_to_grid(mapdata, pose.pose.position))

            # for p in list_of_grid_cells:
            #     print(p)

            # self.orig_pub.publish(plan.makeDisplayMsg(plan, mapdata, list_of_grid_cells))

            if not resp.plan.poses:
                print("ERROR ERROR ERROR TRY SOMETHINGS ELSEEEEEE")
                return None
            else:
                return resp.plan.poses


            #return resp.plan.poses

        except rospy.ServiceException as e:
            print("Service call has failed: %s"%e)
            return None
    
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
        print("GIVEN X: " + str(wp.x))
        print("GIVEN WORLD ORIGIN: " + str(world_origin_x))
        print("MAP RESOLUTION: " + str(map_resolution))
        print("CURRENT CELL: " + str(cell_position))
        

        return cell_position
        
    def update_odometry(self, msg: Odometry):
       
        self.px = msg.pose.pose.position.x 
        self.py = msg.pose.pose.position.y 
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

        

        try:
            position, quaternion = self.listener.lookupTransform("/map",  "/base_link", rospy.Time())
            location =Point(x=position[0], y=position[1])
            orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.px = location.x
            self.py = location.y
            self.pth = yaw
        except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
            print("Not working")
            pass

    #GO_TO WAITS FOR POSESTAMPED MESSAGE AND THEN MOVES TO DESIRED XYZ POSITION AND FINAL ROTATION
    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        rospy.wait_for_message("/odom", Odometry)
        #rospy.loginfo(msg)
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y 
        delta_y = target_y - self.py
        delta_x = target_x - self.px 
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        angle_to_pose = atan2(delta_y, delta_x)
        rospy.loginfo("target x = " + str(target_x) + " target y = " + str(target_y))
        rospy.loginfo("current x = " + str(self.px) + " current y = " + str(self.py))
        rospy.loginfo("delta x = " + str(delta_x) + " delta y = " + str(delta_y))
        
        # Rotate to look at target location
        self.rotate(angle_to_pose, 0.5)
        print("rotation 1 complete!")
        rospy.sleep(0.01)

        # Drive to target location
        
        distance_to_target = abs(sqrt(pow(delta_y, 2) + (pow(delta_x, 2))**2))
        
        self.drive(distance_to_target, 0.13)
        print("Reached target location!")
        rospy.sleep(0.01)



        # Rotate to target orientation
        # self.rotate(yaw, 0.4)
        # print("Reached target pose!")



        print("")
        rospy.loginfo("target x = " + str(target_x) + " target y = " + str(target_y))
        rospy.loginfo("current x = " + str(self.px) + " current y = " + str(self.py))
        print("")
        print("")

 
        


    #DRIVE TELLS THE TURTLEBOT TO GO STRAIGHT FOR A SPECIFIED DISTANCE AND SPEED
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        # wait until a new odometry msg is recived
        print(f'in drive, distance target: {distance}')
        rospy.wait_for_message("/odom", Odometry)
        initialPose_x = self.px
        initialPose_y = self.py
        distanceTolerance = 0.04
        curr_distance = 0.0
        rate = rospy.Rate(10) # Publish rate of 10Hz
        while (not rospy.is_shutdown()) and (abs(distance - curr_distance) >= distanceTolerance):
        # Euclidean distance difference - "error"
        
            self.send_speed(linear_speed, 0.0)
            rate.sleep()
            curr_distance = abs(sqrt(pow(self.py - initialPose_y, 2 ) + (pow(self.px - initialPose_x, 2))**2))
            #rospy.loginfo(f'distance to target: {abs(distance - curr_distance)}')
        self.send_speed(0.0, 0.0)
        print("reached target distance")



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':

    FrontierNodeClient().run()
    
