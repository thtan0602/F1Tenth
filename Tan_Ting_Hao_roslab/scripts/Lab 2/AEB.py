#!/usr/bin/env python
import rospy
import numpy as np
import math
from math import sin, cos, pi
from sensor_msgs.msg import LaserScan
#from Tan_Ting_Hao_roslab.msg import LaserScan_Data_for_TTC
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

# TODO: import ROS msg types and libraries

class Safety(object):
    v_x = 0
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
 	    self.LaserScan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.brake_publisher = rospy.Publisher("brake", AckermannDriveStamped, queue_size = 1)
        self.brake_bool_publisher = rospy.Publisher("brake_bool", Bool, queue_size = 1)
        self.brake_bool_msg = Bool()
        self.brake_msg = AckermannDriveStamped()

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg
	    self.v_x = self.speed.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        self.scan_info = scan_msg
        TTC_threshold = 1.2
        TTC_min = 80
        v_x = self.v_x
	    brake_bool_msg = self.brake_bool_msg
	    brake_msg = self.brake_msg

	    for i in range(len(self.scan_info.ranges)):
            distance = self.scan_info.ranges[i]
	    
            if not (np.isnan(distance) or np.isinf(distance)):
               angle = self.scan_info.angle_min + self.scan_info.angle_increment * i
               derivative_of_distance = v_x * math.cos(angle)
               
	# TODO: calculate TTC
               if (derivative_of_distance > 0 and distance/derivative_of_distance < TTC_min):
	               TTC_min = distance/derivative_of_distance
                   print(TTC_min)
        

        # TODO: publish brake message and publish controller bool
                   if (TTC_min <= TTC_threshold):
                        brake_bool_msg.data = True
                        brake_msg.drive.speed = 0
                        print("OBSTACLES AT", angle*180/math.pi, "deg")
                        print("TTC_min:", TTC_min)
                        self.brake_publisher.publish(brake_msg)
                        self.brake_bool_publisher.publish(brake_bool_msg)
                    else:
                        brake_bool_msg.data = False

def main():
    rospy.init_node('Tan_Ting_Hao_safety')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
 
