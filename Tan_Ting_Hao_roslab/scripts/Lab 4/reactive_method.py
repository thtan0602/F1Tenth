#!/usr/bin/env python
from __future__ import print_function

import math
import sys

# ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.min_index = 0
        self.max_index = 0
        self.proc_ranges = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.proc_ranges = ranges.ranges
        for idx in range(len(self.proc_ranges)):
            if math.isnan(self.proc_ranges[idx]):
                self.proc_ranges[idx] = 0.0
            elif math.isinf(self.proc_ranges[idx]):
                self.proc_ranges[idx] = 10.0

        i = 0
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
    
    def find_max_gap(self, free_space_ranges):
        """ Return the start_idx idx & end_idx idx of the max gap in free_space_ranges
        """
        start_idx = self.min_index
        end_idx = self.min_index
        current_start = self.min_index - 1
        current_length = 0
        max_length = 0
        
        for idx in range(int(self.min_index), int(self.max_index + 1)):
            if current_start < self.min_index:
                if free_space_ranges[idx] > 0.0:
                    current_start = idx
            elif free_space_ranges[idx] <= 0.0:
                current_length = idx - current_start
                if current_length > max_length:
                    max_length = current_length
                    start_idx = current_start
                    end_idx = idx - 1
                current_start = self.min_index - 1
        if current_start >= self.min_index:
            current_length = self.max_index + 1 - current_start
            if current_length > max_length:
                max_length = current_length
                start_idx = current_start
                end_idx = self.max_index

        return start_idx, end_idx
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start_idx and end_idx indicies of max-gap range, respectively
        Return idx of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        current_max = 0
        best_point_index = 0
        
        for idx in range(int(start_i), int(end_i + 1)):
            if ranges[idx] > current_max:
                best_point_index = idx      
                 
        return idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.proc_ranges = self.preprocess_lidar(data)

        min_angle = -60 / 180.0 * math.pi
        self.min_index = math.floor((min_angle - data.angle_min) / data.angle_increment)
        max_angle = 60 / 180.0 * math.pi
        self.max_index = math.ceil((max_angle - data.angle_min) / data.angle_increment)
        edited_range = []
        for idx in range(int(self.min_index), int(self.max_index) + 1):
            edited_range.append(idx)

        # Find closest point to LiDAR
        closest_pt_distance = 100
        closest_pt_index = 0

        for idx in edited_range:
            if self.proc_ranges[idx] < closest_pt_distance:
                closest_pt_distance = self.proc_ranges[idx]
                closest_pt_index = idx

        # Eliminate all points inside 'bubble' (set them to zero)
        radius = 180
        for idx in range(int(closest_pt_index - radius), int(closest_pt_index + radius + 1)):
            self.proc_ranges[idx] = 0.0

        # Find max length gap
        start_idx, end_idx = self.find_max_gap(self.proc_ranges)

        # Find the best point in the gap ( furthest )
        angle = 0.0
        best_point_index = self.find_best_point(start_idx, end_idx, self.proc_ranges)
        angle = data.angle_min + best_point_index * data.angle_increment

        velocity = 3.0
        if abs(angle) > 1 / 9 * math.pi:
            velocity = velocity * (1.0 / 3)
        elif abs(angle) > 1 / 18 * math.pi:
            velocity = velocity * (2.0 / 3)
        else:
            velocity = velocity

        # Publish Drive message
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
