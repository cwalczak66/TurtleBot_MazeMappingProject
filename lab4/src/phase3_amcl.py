#!/usr/bin/env python3
from __future__ import annotations
from queue import Empty
#from statistics import covariance

from attr import s
#from lab2.src.lab2 import Lab2
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, TransformStamped, Twist, Vector3
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
        rospy.Subscriber('initialpose' , PoseWithCovarianceStamped , self.amcl_move)
        rospy.Subscriber('bool_topic', Bool, self.wait_for_waypoint)
        


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
    
    def amcl_move(self, msg:PoseWithCovarianceStamped):

        number_of_turns = 0 # Start at 0 turns done
        #("rosservice call /global_localization") # Scatter a bunch of potential robot positions around the map for amcl to sort through

        # Do a minimum number of turns and then keep turning until we are very confident about our location or we've turned for too long\
        #WANT COVARIANCE TO BE SMALL

        rospy.wait_for_service('global_localization')
        rospy.ServiceProxy('global_localization', Empty)





        print("")
        print("default cov: " + str(self.covariance))
        while (number_of_turns < 10 and self.covariance > 0.01):
            print("covariance value" + str(self.covariance))
            self.rotate(pi / 2, 0.1)
            number_of_turns += 1
            rospy.sleep(.75)
        
        # Moving the robot to the goal once localized

        rospy.loginfo("Localized. Driving to goal")
        go_to_msg = PoseStamped()
        go_to_msg.header.frame_id = "map"
        go_to_msg.header.stamp = rospy.Time.now
        go_to_msg.pose = msg.pose.pose

        mapdata = self.request_map()
        
        waypoints = self.get_astar_path(mapdata, go_to_msg)
      

        for waypoint in waypoints:
            self.go_to_pub.publish(waypoint)
            print("waiting")
            rospy.wait_for_message('bool_topic', Bool)



        self.go_to_pub.publish(go_to_msg)
        #PathPlannerClient.path_planner_client(self, msg)
    
    #Update the covariance when receiving a message from amcl_pose
    #Covariance is the sum of the diagonal elements of the covariance matrix

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
            path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
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
        print("CURRENT X WORLD: "+str(wp.x))
        wp.y = self.py
        grid_robot = self.world_to_grid(mapdata, wp)
        
        start.pose.position.x = grid_robot[0]
        start.pose.position.y = grid_robot[1]
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time.now()
       
        

        try:
            path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
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


            return resp.plan.poses

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
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        trans = [0,0]
        rot = [0,0,0,0]
        try:
            (trans,rot) = self.listener.lookupTransform('/map','/base_footprint',rospy.Time(0)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("HEY I DIDN'T WORK")
        self.px = trans[0]
        self.py = trans[1]

        quat_list = [rot[0], rot[1], rot[2], rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """

        rospy.spin()


        
if __name__ == '__main__':
    FrontierNodeClient().run()
    
