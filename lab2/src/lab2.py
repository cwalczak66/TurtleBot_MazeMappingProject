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
        
        self.px = 0
        self.py = 0

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
        # TODO
        msg = Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed))
        ### Publish the message
        self.cmd_pub.publish(Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed)))
        # TODO
        rate = rospy.Rate(10) # Publish rate of 10Hz
    
        
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        # saving initial pose
        initialPose_x= self.px
        initialPose_y = self.py
        distanceTolerance = 0.5
        # initial_pose = [[initialPose_x],[initialPose_y]]
        currentPose_x = self.px
        currentPose_y = self.py
        Kp = 0.1
        
        while not rospy.is_shutdown():
        # Proportional control
        # Euclidean distance difference - "error"
            distance = abs(sqrt(pow(currentPose_y - initialPose_y, 2 ) + (pow(currentPose_x - initialPose_x, 2))**2))
            linear_speed = Kp * distance

            
            # print(type(distance))
            # print(type(distanceTolerance))
            if distance <= distanceTolerance:
                linear_speed = 0.0
            else:
                rospy.sleep(0.1)
            self.send_speed(linear_speed,0)
        


    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        target_yaw = self.pth + angle
        ang_tol = 0.01
        angle_difference = target_yaw - self.pth

        # Normalizing angle difference to range btw pi and -pi
        angle_difference = atan2(sin(angle_difference), cos(angle_difference))
        
        while not rospy.is_shutdown():

            if angle_difference <= ang_tol:
                self.send_speed(0,0)
            else:
            # Normalizing angle difference to range btw pi and -pi
                if angle_difference > 0:
                    self.send_speed(0, aspeed )
                    print("clockwise")
                    print(angle_difference)
                else:
                    self.send_speed(0, -aspeed)
                    print("anti-clockwise")


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
        current_angle = self.pth
        current_x = self.px
        current_y = self.py
        target_x = PoseStamped.pose.position.x
        target_y = PoseStamped.pose.position.y
        delta_y = target_y - current_y
        delta_x = target_x - current_x
        
        angle_to_pose = atan2(delta_y, delta_x)
        target_angle = current_angle + PoseStamped.pose.orientation.w
        angle_diff = target_angle - current_angle

        while not rospy.is_shutdown():

            # Rotate to look at target location
            self.rotate(angle_to_pose, 0)
            print("rotation 1 complete!")

            # Drive to target location
            distance_to_target = abs(sqrt(pow(delta_y, 2 ) + (pow(delta_x, 2))**2))
            linear_speed = Kp * distance_to_target
            self.drive(distance_to_target, linear_speed)
            print("Reached target location!")

            # Rotate to target orientation
            self.rotate(angle_diff, 0)
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
        
        # rospy.loginfo("Hola")



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
       # self.drive(1.0,0)
       # self.rotate(pi/2,1) 
        self.go_to(msg=PoseStamped)  
         
        rospy.spin()
        

if __name__ == '__main__':
    Lab2().run()
    
    