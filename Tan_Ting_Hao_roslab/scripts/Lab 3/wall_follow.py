#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.00
kd = 0.001
ki = 0.005
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_time = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.9
VELOCITY = 1.5 # meters per second
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    a = 0
    b = 0
    alpha = 0
    thetha_a = 0
    thetha_b = 0
    d_t = 0
    future_d_t = 0
    """ Implement Wall Following on the car
    """
    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        if angle >= -45 and angle <= 225:
            index = len(data) * (angle + 90) / 360
            if not np.isnan(int(index)) and not np.isinf(int(index)):
                return data[int(index)]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        angle = 0.0
        current_time = rospy.get_time()
        dt = current_time - prev_time
        #TODO: Use kp, ki & kd to implement a PID controller for 
        integral += prev_error * dt
        derivative = (error - prev_error) / dt

        angle = kp * error + ki * integral + kd * derivative
        prev_error = error
        prev_time = current_time

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            drive_msg.drive.speed = velocity
        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        self.thetha_a = 125
        self.thetha_b = 180
        theta = math.radians(abs(self.thetha_a - self.thetha_b))

        self.a = self.getRange(data, self.thetha_a)
        self.b= self.getRange(data, self.thetha_b)

        self.alpha = math.atan2(self.a * math.cos(theta) - self.b, self.b * math.sin(theta))

        self.d_t = self.b * math.cos(self.alpha)
        self.future_d_t = self.d_t + CAR_LENGTH * math.sin(self.alpha)

        return leftDist - self.future_d_t

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT) #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)