#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1) #TODO

        self.proc_ranges = 0
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.proc_ranges = list(ranges.ranges)
        '''for idx in range(len(self.proc_ranges)):
            if math.isnan(self.proc_ranges[idx]):
                self.proc_ranges[idx] = 0.0
            elif math.isinf(self.proc_ranges[idx]):
                self.proc_ranges[idx] = 3.0

        window_size = 3
        average_ranges = []
        for idx in range(len(self.proc_ranges)):
            if idx < len(self.proc_ranges) - window_size + 1:
                this_window = self.proc_ranges[idx: idx + window_size]
                window_average = sum(this_window) / window_size
                average_ranges.append(window_average)
            else:
                this_window = self.proc_ranges[idx: idx + window_size]
                window_average = sum(this_window) / window_size
                average_ranges.append(window_average)

        return average_ranges
        '''

        for index in range(len(self.proc_ranges)):
            if math.isnan(self.proc_ranges[index]):
                self.proc_ranges[index] = 0.0
            elif math.isinf(self.proc_ranges[index]):
                self.proc_ranges[index] = 10.0

        i = 0
        window_size = 3
        average_ranges = []
        while i < len(self.proc_ranges) - window_size + 1:
            this_window = self.proc_ranges[i: i + window_size]
            window_average = sum(this_window) / window_size
            average_ranges.append(window_average)
            i += 1

        return average_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        count = 0
        prev = 0
        indexend = 0

        for idx in range(0,len(free_space_ranges)):
            if free_space_ranges[idx] != 0:
                count += 1
            else:            
                if count > prev:
                    prev = count
                    indexend = idx
                count = 0
        
        end_index = indexend - 1
        start_index = end_index - prev + 1

        return start_index, end_index
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return np.argmax(ranges[start_i : end_i])

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.proc_ranges = self.preprocess_lidar(data)
        
        #Find closest point to LiDAR
        closest_pt_distance = min(self.proc_ranges)
        closest_pt_index = np.argmin(self.proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero) 
        safety_bubble_radius = 10
        for idx in range(len(self.proc_ranges)):
            if math.sqrt(self.proc_ranges[idx] ** 2 + self.proc_ranges[closest_pt_index] ** 2 - 2 * self.proc_ranges[idx] * self.proc_ranges[closest_pt_index] * math.cos(data.angle_increment * abs(closest_pt_index - idx))) < safety_bubble_radius:
                self.proc_ranges[idx] = 0   
       
        #Find max length gap 
        start_index, end_index = self.find_max_gap(self.proc_ranges)
       
        #Find the best point in the gap 
        best_point_index = self.find_best_point(start_index, end_index, self.proc_ranges) + start_index

        #Publish Drive message
        current_max = 0.0
        angle = 0.0
        for idx in range(start_index, end_index + 1):
            if self.proc_ranges[idx] > current_max:
                current_max = self.proc_ranges[idx]
                angle = data.angle_min + idx * data.angle_increment
                print(angle)
        #angle = data.angle_min + data.angle_increment * best_point_index

        base_velocity = 1.5

        if abs(angle) > 1 / 9 * math.pi:
            velocity = base_velocity * (1.0 / 3)
        elif abs(angle) > 1 / 18 * math.pi:
            velocity = base_velocity * (2.0 / 3)
        else:
            velocity = base_velocity

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        
def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)