#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
from math import sqrt , pi, atan2, sin, cos 
from std_msgs.msg import Bool
import tf
from tf import TransformListener
from geometry_msgs.msg import Quaternion
import time


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('Lab2')

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        self.done_move_pub = rospy.Publisher('bool_topic', Bool, queue_size=10)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        #self.xpose = PoseStamped.pose #.position.x
        #print(self.xpose)
        self.px = 0
        self.py = 0
        self.kp = 0.1
        
        # yaw angle
        self.pth = 0  

        self.listener = TransformListener()

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
        current_time = time.time()
        
    
        end_time = current_time+11

        print(f'in drive, distance target: {distance}')
        rospy.wait_for_message("/odom", Odometry)
        initialPose_x = self.px
        initialPose_y = self.py
        distanceTolerance = 0.01
        curr_distance = 0.0
        rate = rospy.Rate(10) # Publish rate of 10Hz
        while (not rospy.is_shutdown()) and (abs(distance - curr_distance) >= distanceTolerance) and (current_time < end_time):
        # Euclidean distance difference - "error"
            current_time = time.time()
            print("current " + str(current_time))
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
        self.rotate(angle_to_pose, 0.25)
        print("rotation 1 complete!")
        rospy.sleep(0.1)

        # Drive to target location
        
        distance_to_target = abs(sqrt(pow(delta_y, 2) + (pow(delta_x, 2))**2))
        
        self.drive(distance_to_target, 0.1)
        print("Reached target location!")
        rospy.sleep(0.1)



        # Rotate to target orientation
        # self.rotate(yaw, 0.4)
        # print("Reached target pose!")



        print("")
        rospy.loginfo("target x = " + str(target_x) + " target y = " + str(target_y))
        rospy.loginfo("current x = " + str(self.px) + " current y = " + str(self.py))
        print("")
        print("")

        bool_msg = Bool()
        bool_msg.data = True
        self.done_move_pub.publish(bool_msg)


# UPDATES ROBOT CURRENT POSITION AND ORIENTATION SO OTHER FUNCTIONS KNOW WHERE TB IS IN REAL TIME
    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
        
        

        try:
            position, quaternion = self.listener.lookupTransform("/map",  "/base_footprint", rospy.Time())
            location =Point(x=position[0], y=position[1])
            orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.px = location.x
            self.py = location.y
            self.pth = yaw
        except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
            print("Not working")
            pass

# TO BE IMPLEMENTED
    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code




    def run(self):
        #self.drive(1.0,2.0)
        #self.rotate(0.0,0.5) 
        #self.drive(0.6, 0.5) 
        rospy.spin()
        

if __name__ == '__main__':
    Lab2().run()
    