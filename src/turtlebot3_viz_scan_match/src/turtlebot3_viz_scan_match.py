#!/usr/bin/env python3
import rospy
import numpy as np
import math

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Visualization():

    def __init__(self):
        self.map = None
        self.odom = None

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def publish_to_topics(self):
        self.pub_gt = rospy.Publisher("/scan_match/new_pose", Odometry, queue_size=1)

    def map_callback(self, msg):
        self.map = msg 

    def odom_callback(self, msg):
        self.odom = msg 

    def scan_callback(self, msg):
        pass 


if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()
    rospy.spin()


