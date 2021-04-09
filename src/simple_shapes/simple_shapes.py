#!/usr/bin/env python

# import of python modules
import math
import numpy as np

# import of relevant libraries
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# import custom modules
from aux_function import *

# Constants
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ERROR_TOPIC = 'error'
DEFAULT_ODOM_TOPIC = 'odom'

"""
SimpleShapes class definition to achieve a task
"""
class SimpleShapes():
    def __init__(self, dir_counter_clock):
        """ Constructor """
        # setting up publishers/subscribers 
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self._error_pub = rospy.Publisher(DEFAULT_ERROR_TOPIC, Float32, queue_size=1)
        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_call_back, queue_size=1)

        # initial pose data by subscription once
        self.initial_pose_x = None
        self.initial_pose_y = None

        # get params from .yaml file via param server
        self.linear_velocity = rospy.get_param('~LINEAR_VELOCITY')
        self.angular_velocity = rospy.get_param('~ANGULAR_VELOCITY') * math.pi
        self.rate = rospy.Rate(rospy.get_param('~FREQUENCY'))
        self.error_rate = rospy.Rate(rospy.get_param('~ERROR_FREQUENCY'))

        # other properties to achieve a task
        self.coordinate_list = None # polygon cooridnate list where the robot should calculate the error
        self.dir_counter_clock = dir_counter_clock # param received from launching the node
        self.vertex_arrival = False # flag for vertex arrival status to publish error
        self.iter = 0 # iteration of the coordinates
        self.odom_msg = None

    def move_forward(self, distance):
        """ 
        Function to move forward for a given distance 
        """
        # next_wp
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        # TODO: while sentence by distance

        start_time = rospy.get_rostime()
        duration = rospy.Duration(distance/twist_msg.linear.x)

        while not rospy.is_shutdown():
            # Check if traveled of given distance based on time.
            if rospy.get_rostime() - start_time >= duration:
                break

            else:
                self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set publishing frequency.
            self.rate.sleep()

        # Traveled the required distance, stop.
        self.stop()

    def rotate_in_place(self, rotation_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        depending on the rotation direction (clock vs counter-clock)
        """
        twist_msg = Twist()

        if self.dir_counter_clock: # counter-clockwise True
            twist_msg.angular.z = self.angular_velocity
        else: # counter-clockwise False, i.e., clockwise
            twist_msg.angular.z = - self.angular_velocity
        
        duration = rotation_angle / abs(twist_msg.angular.z)
        start_time = rospy.get_rostime()

        while not rospy.is_shutdown():
            # Check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                break
                
            # Publish message.
            self._cmd_pub.publish(twist_msg)
            
            # Sleep to keep the set frequency.
            self.rate.sleep()

        # Rotated the required angle, stop.
        self.stop()


    def stop(self):
        """
        function to stop the robot by linear and angular velocity all zeros
        """
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def update_pose(self):
        """
        function to update the pose message when the robot was booted at the initial position
        """
        msg = rospy.wait_for_message(DEFAULT_ODOM_TOPIC, Odometry)
        self.initial_pose_x = msg.pose.pose.position.x
        self.initial_pose_y = msg.pose.pose.position.y

        print("Robot boot & Initial robot pose received! \n x: {}, y: {}".format(
            self.initial_pose_x, self.initial_pose_y))

    def first_heading_match(self):
        """
        function to match the robot heading with the 2nd vertex
        it returns +, - angle (for my double check) depending on check vector function (aux_function module)
        However, rotation_in_place function will deal with the turning direction by just this return as abs value.
        """

        # 1. get yaw angle of the robot when it reachsed the first vertex
        msg = rospy.wait_for_message(DEFAULT_ODOM_TOPIC, Odometry)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print("yaw: {}".format(yaw))

        # 2. two vectors to find out between angles
        # base vector
        heading_vec = np.array([math.cos(yaw), math.sin(yaw)])

        # vertex vector (angle should be found between this and base vector)
        vertex_vec = np.array([self.coordinate_list[1][0]-self.coordinate_list[0][0],
                    self.coordinate_list[1][1]-self.coordinate_list[0][1]])

        print("heading vec: {}".format(heading_vec))
        print("vertex vec: {}".format(vertex_vec))
        between_angle = check_vector_angle(heading_vec, vertex_vec)
        print("between angle: {}".format(math.degrees(between_angle)))

        return between_angle

    def _odom_call_back(self, msg):
        """
        odom call back function to save odom data into the class property (used in error broadcast)
        """
        self.odom_msg = msg

    def error_broadcast(self):
        """
        function to publish error based on the discrepacy between the ideal vertex and actual robot position by odom
        Note: it only publishes while the robot's vertex_arrival flag is True
        """
        if self.vertex_arrival:  # robot at the vertex
            # error message initialization with Float32 std_msg
            error_msg = Float32()
            error_msg.data = distance_calculator(self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y,
                                                self.coordinate_list[self.iter][0], self.coordinate_list[self.iter][1])
            self._error_pub.publish(error_msg)
            self.error_rate.sleep()

            print("odom position : {}".format((self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y)))
            print("ideal coordinate : {}".format((self.coordinate_list[self.iter][0], self.coordinate_list[self.iter][1])))
            print("error: {}".format(error_msg.data))

        else:  # robot not at the vertex
            pass

    def shape_drawer(self, side_number, side_length, dir_counter_clock):
        """
        shape drawing task function as per the user inputs
        """
        # figure out the regular polygon needed to be drawn
        self.coordinate_list = regular_polygon(side_number, side_length, dir_counter_clock)

        # starting to follow the polygon
        while self.iter < len(self.coordinate_list):
            self.vertex_arrival = False

            if self.iter == 0 :
                print("-------------- approaching no. {} vertex --------------".format(self.iter))
                # find initial distance to travel from the starting point
                initial_dist = distance_calculator(self.initial_pose_x, self.initial_pose_y,
                                            self.coordinate_list[self.iter][0], self.coordinate_list[self.iter][1])
                print("initial dist {}".format(initial_dist))

                # go to the first vertex from the starting point
                self.move_forward(initial_dist) 

                # match the heading
                self.rotate_in_place(abs(self.first_heading_match()))

                # # arrived at vertex
                # self.vertex_arrival = True
                # rospy.sleep(2) # wait for a while to publish error on this vertex
                # self.iter += 1
                print("Robot on the first vertex and mached heading to the first side")
                

            else:
                # drawing the regular shape 
                print("-------------- approaching no. {} vertex --------------".format(self.iter))
                if self.iter != len(self.coordinate_list) - 1: # all except for the last vertex
                    
                    # follow the side
                    print("proceed: {} meter".format(side_length))
                    self.move_forward(side_length)  
                    
                    # next vertex heading match
                    print("rotate: {} degrees".format(math.degrees(abs(2* math.pi/side_number))))
                    self.rotate_in_place(abs(2* math.pi/side_number))  

                    # arrived at vertex
                    self.vertex_arrival = True
                    rospy.sleep(2) # wait for a while to publish error on this vertex

                else: # go to the last vertex / no need to do additional rotation at the final step
                    
                    # follow the side
                    self.move_forward(side_length)  
                    print("proceed: {} meter".format(side_length))

            # arrived at vertex
            self.vertex_arrival = True # flag change
            self.error_broadcast()  # error broad casting
            rospy.sleep(2) # wait for a while to publish error before moving on

            # next coordinate
            self.iter += 1
                
                    
        # ensure one more protection for stop
        self.stop()
        print("---------------------------------------")
        rospy.loginfo("the robot achived the task!")