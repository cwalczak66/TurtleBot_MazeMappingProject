#!/bin/env python3

from __future__ import annotations
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from geometry_msgs.msg import Twist, Vector3
from path_planner import PathPlanner
from tf.transformations import euler_from_quaternion
from math import sqrt , pi, atan2, sin, cos


class PathPlannerClient:

    def __init__(self):
        # suscribing to 2d nav goal 
        rospy.init_node("path_planner_client")
        print("starting client")

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.path_planner_client)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        self.start_pose = PoseStamped()


        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
    #    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #self.xpose = PoseStamped.pose #.position.x
        #print(self.xpose)
        self.px = 0
        self.py = 0
        self.kp = 0.1
        
        # yaw angle
        self.pth = 0  




    def path_planner_client(self, msg: PoseStamped): 
    
        rospy.wait_for_service('plan_path')
        start = self.start_pose
        rospy.loginfo(msg)
        goal = PoseStamped()
        goal.pose = msg.pose
        goal.header = msg.header

        try:
            path_planner_call = rospy.ServiceProxy('plan_path', GetPlan)
            resp = path_planner_call(start, goal, 0)
            #rospy.loginfo(resp.plan.poses)
            step = 1
            for waypoint in resp.plan.poses:

                
                print("STEP "+ str(step))
                print("")
                self.go_to(waypoint)
                step = step + 1
                
                print("FINISHED GO_TO")

            return resp

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    
    
    # UPDATES ROBOT CURRENT POSITION AND ORIENTATION SO OTHER FUNCTIONS KNOW WHERE TB IS IN REAL TIME
    def update_odometry(self, odom_msg: Odometry):
        
        
        self.px = odom_msg.pose.pose.position.x + 4.8
        self.py = odom_msg.pose.pose.position.y + 4.8
        quat_orig = odom_msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

        #print("x: " + str(self.px) + "y: " + str(self.py) + "yaw: " + str(self.pth))




        # pose_stamped_msg = PoseStamped()
        # pose_stamped_msg.header = odom_msg.header
        # pose_stamped_msg.pose = odom_msg.pose.pose
        # self.start_pose = pose_stamped_msg
        

#SENDS DESIRED MOTOR SPEED AS A TWIST MSG
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
        distanceTolerance = 0.1
        curr_distance = 0.0
        rate = rospy.Rate(10) # Publish rate of 10Hz
        while (not rospy.is_shutdown()) and (abs(distance - curr_distance) >= distanceTolerance):
        # Euclidean distance difference - "error"
            self.send_speed(linear_speed, 0.0)
            rate.sleep()
            curr_distance = abs(sqrt(pow(self.py - initialPose_y, 2 ) + (pow(self.px - initialPose_x, 2))**2))
            rospy.loginfo(f'distance to target: {abs(distance - curr_distance)}')
        self.send_speed(0.0, 0.0)
        print("reached target distance")

    

#ROTATE TELLS TURTLEBOT TO ROTATE TO DESIRED ANGLE
    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        ang_tol = 0.08
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
        self.rotate(angle_to_pose, 0.1)
        print("rotation 1 complete!")
        rospy.sleep(1)

        # Drive to target location

        distance_to_target = abs(sqrt(pow(delta_y, 2) + (pow(delta_x, 2))**2))
        self.drive(distance_to_target, 0.1)
        print("Reached target location!")
        rospy.sleep(0.7)



        # Rotate to target orientation
        self.rotate(yaw, 0.1)
        print("Reached target pose!")


        print("")
        rospy.loginfo("target x = " + str(target_x) + " target y = " + str(target_y))
        rospy.loginfo("current x = " + str(self.px) + " current y = " + str(self.py))
        print("")
        print("")

    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
       
        rospy.spin()


        
if __name__ == '__main__':
    PathPlannerClient().run()

    
