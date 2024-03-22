#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from cmath import sqrt

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
        rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # TODO
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # TODO
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        
        self.px
        self.py
        self.pth


    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        # TODO
        msg = Twist(linear=Vector3(x=2.0), angular=Vector3(z=0.0))
        ### Publish the message
        # TODO
        self.cmd_vel.publish(Twist(linear=Vector3(x=2.0), angular=Vector3(z=0.0)))
        rate = rospy.Rate(10) # Publish rate of 10Hz
    
        
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        # saving initial pose
        initialPose_x= 0
        initialPose_y = 0
        distanceTolerance = 0.5
        # initial_pose = [[initialPose_x],[initialPose_y]]
        currentPose_x = self.px
        currentPose_y = self.py
        Kp = 0.1
        
        while not rospy.is_shutdown():
        # Proportional control
        # Euclidean distance difference - "error"
            distance = sqrt (pow(currentPose_y - initialPose_y, 2 ) + (pow(currentPose_x - initialPose_x, 2))**2)
            linear_speed = Kp * distance

        
            if distance <= distanceTolerance:
                linear_speed = 0.0
            else:
                rospy.sleep(100)

        initialPose_x = currentPose_x
        initialPose_y = currentPose_y
        


    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        pass # delete this when you implement your code



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
        
        rospy.loginfo("Hola")



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
        self.drive(1.0,0.0)
        rospy.spin()
        

if __name__ == '__main__':
    Lab2().run()
    
    
