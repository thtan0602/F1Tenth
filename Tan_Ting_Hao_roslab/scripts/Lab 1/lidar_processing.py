#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

from Tan_Ting_Hao_roslab.msg import scan_range
import rospy
import numpy as np
import pandas as pd
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

lidar_range = []
lidar_min = 0
lidar_max = 0

def callback(lidar_msg):

    if not (pd.isnull(lidar_msg)):

	global lidar_range
	lidar_range.append(lidar_msg)

def listener():

    sub = rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():

    if(len(lidar_range)) > 0:
	lidar_min = min(lidar_range)
	lidar_max = max(lidar_range)
    message = scan_range()

    while not rospy.is_shutdown():
        pub_closest.publish(lidar_min)
	pub_farthest.publish(lidar_max)
    	message.min_num = lidar_min
	message.max_num = lidar_max
	pub_closest_farthest.publish(message)
	rospy.loginfo(pub_closest, pub_farthest)
        rate.sleep()

if __name__ == '__main__':

    rospy.init_node("lidar_processing", anonymous=True)
    pub_closest = rospy.Publisher("closest_point", Float64, queue_size = 1)
    pub_farthest = rospy.Publisher("farthest_point", Float64, queue_size = 1)
    pub_closest_farthest = rospy.Publisher("scan_range", scan_range, queue_size = 1)
    listener()
    talker()
