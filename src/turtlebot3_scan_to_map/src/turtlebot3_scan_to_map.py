#!/usr/bin/env python3
import rospy

import math
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

class Visualization():

    def __init__(self):
        self.odom = None

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_scan = rospy.Subscriber("/odom", Odometry, self.odom_cammback)

    def publish_to_topics(self):
        self.pub_gt = rospy.Publisher("/scan/gridmap", OccupancyGrid, queue_size=1)

    def odom_callback(self, msg):
        self.odom = msg

    def map_callback(self, msg):
        grid_map = OccupancyGrid()
        meta = MapMetaData()

        grid_map.header = msg.header
        meta.map_load_time = rospy.Time().now()
        meta.origin = self.odom.pose.pose

        
        resolution = 0.01
        width = 500
        sheight = 500



if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()
    rospy.spin()


