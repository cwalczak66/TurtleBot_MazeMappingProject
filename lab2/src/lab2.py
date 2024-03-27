#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
from math import sqrt , pi, atan2, sin, cos 

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('Lab2')
        # TODO
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # TODO
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        
        # TODO
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        #self.xpose = PoseStamped.pose #.position.x
        #print(self.xpose)
        self.px = 0
        self.py = 0
        self.kp = 0.1
        # yaw angle
        self.pth = 0  

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        
        msg = Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed))
        #print(f"message sending, rot= {angular_speed} lin= {linear_speed}")
        ### Publish the message
        self.cmd_pub.publish(msg)
         
        
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
        #Kp = 0.1
        # initial_pose = [[initialPose_x],[initialPose_y]]
        # currentPose_x = 1.0
        # currentPose_y = 1.0
        # Kp = self.kp
        #curr_distance = sqrt(pow(self.py - initialPose_y, 2 ) + (pow(self.px - initialPose_x, 2))**2)
        curr_distance = 0.0
        
        rate = rospy.Rate(10) # Publish rate of 10Hz
        while (not rospy.is_shutdown()) and (abs(distance - curr_distance) >= distanceTolerance):
        # Proportional control
        # Euclidean distance difference - "error"
            #linear_speed = Kp * distance
            self.send_speed(linear_speed, 0.0)
            rate.sleep()
            curr_distance = abs(sqrt(pow(self.py - initialPose_y, 2 ) + (pow(self.px - initialPose_x, 2))**2))
            rospy.loginfo(f'remaining distance: {abs(distance - curr_distance)}')
            #rospy.loginfo()
            # print(type(distance))
            # print(type(distanceTolerance))
            #rospy.sleep(0.5)
        self.send_speed(0.0, 0.0)
        print("reached distance goal")

    

#ROTATE TO ANGLE DEPENDS ON ROBOT POSE
    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        #target_yaw = self.pth + angle
        ang_tol = 0.1
        rospy.wait_for_message("/odom", Odometry)
        rate = rospy.Rate(10) # Publish rate of 10Hz
        while not rospy.is_shutdown():
            angle_difference = (angle - self.pth)% (2* pi)
            print(f'target angle: {angle} current angle: {self.pth} angle difference: {angle_difference}')

            # Normalizing angle difference to range btw pi and -pi
            #angle_difference = atan2(sin(angle_difference), cos(angle_difference))
            #print(angle_difference)
            while angle_difference > pi:
                angle_difference = angle_difference - 2 * pi
            while angle_difference < -pi:
                angle_difference = angle_difference + 2*pi
            #print("ready to rotate")
            rate.sleep()
            #print(angle_difference)
            
            if abs(angle_difference) <= ang_tol:
                
                self.send_speed(0.0,0.0)
                rospy.sleep(0.5)
                self.send_speed(0,0)
                
                print("reached goal!")
                break
            else:
                # Normalizing angle difference to range btw pi and -pi
                if angle_difference > 0:    
                    self.send_speed(0, aspeed)
                    #print("clockwise")
                    
                else:
                    self.send_speed(0, -aspeed)
                    #print("anti-clockwise")
                rospy.sleep(0.5)
        
        self.send_speed(0,0)
        print("robot should stop now")
        self.send_speed(0.0,0.0)
        # if abs(self.pth - target_yaw) != ang_tol:
        #     if self.pth - angle > pi:
        #         #self.cmd_pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=1.0)))
        #         self.send_speed(0,1)
        #     else:
        #         #self.cmd_pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=-1.0)))
        #         self.send_speed(0,-1)
        # else:
        #     self.send_speed(0,0)
    



    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
<<<<<<< HEAD
        #print(msg.pose.position)
=======
        print(msg.pose.position)
>>>>>>> 6757ce7c8ef1b9e34c7a61b0e722f2b69680e912
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y 
        delta_y = target_y - self.px
        delta_x = target_x - self.py
<<<<<<< HEAD
        
        
=======
>>>>>>> 6757ce7c8ef1b9e34c7a61b0e722f2b69680e912
        angle_to_pose = atan2(delta_y, delta_x)

        # Rotate to look at target location
        print("TIME FOR ROTATION ONE!")
        self.rotate(angle_to_pose, 0.5)
        print("rotation 1 complete!")
        rospy.sleep(0.7)
        # Drive to target location
        distance_to_target = abs(sqrt(pow(delta_y, 2 ) + (pow(delta_x, 2))**2))
        
        self.drive(distance_to_target, 0.5)
        print("Reached target location!")
        rospy.sleep(0.7)

        # Rotate to target orientation
        self.rotate(msg.pose.orientation.z, 0.5)
        print("Reached target pose!")



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
        
        #rospy.loginfo("Hola")



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
        # while True:
        #     self.send_speed(1,1)
        #self.drive(1.0,2.0)
        #self.rotate(0.0,0.5) 
        #self.drive(0.6, 0.5) 
        rospy.spin()
        

if __name__ == '__main__':
    Lab2().run()
    
    