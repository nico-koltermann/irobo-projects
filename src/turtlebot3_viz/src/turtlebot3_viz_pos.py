#!/usr/bin/env python3
import rospy
import numpy as np
import math

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point, PoseWithCovarianceStamped
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Visualization():

    def __init__(self):
        self.amcl = None
        self.amcl_arr = []

        self.amcl_marker = getEmptyMarker()
        self.amcl_marker.color.g = 1.0

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_amcl_pos = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pos_callback)

    def publish_to_topics(self):
        self.pub_amcl = rospy.Publisher("/turtlebot3/polygon/amcl", Marker, queue_size=1)

    def amcl_pos_callback(self, msg):
        self.amcl = msg

    def run(self):
        if self.amcl != None:

            b = np.array([self.amcl.pose.pose.position.x, self.amcl.pose.pose.position.y])

            self.amcl_arr.append(Point(self.amcl.pose.pose.position.x, self.amcl.pose.pose.position.y, 0))
            
            self.amcl_marker.points = self.amcl_arr

            self.pub_amcl.publish(self.amcl_marker)

def getEmptyMarker():
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # Line
    marker.type = 4

    # Set the pose of the marker
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.scale.x = 0.02
    marker.scale.y = 0.01
    marker.scale.z = 0.4

    return marker

if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()

