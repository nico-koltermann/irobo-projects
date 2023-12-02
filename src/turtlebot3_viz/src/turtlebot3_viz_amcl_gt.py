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
        self.gt = None
        self.gt_arr = []
        self.amcl = None
        self.amcl_arr = []

        self.amcl_marker = getEmptyMarker()
        self.amcl_marker.color.g = 1.0
        self.gt_marker = getEmptyMarker()
        self.gt_marker.color.r = 1.0

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_gt_pos = rospy.Subscriber("/ground_truth/state", Odometry, self.gt_pos_callback)
        self.sub_amcl_pos = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pos_callback)

    def publish_to_topics(self):
        self.pub_gt = rospy.Publisher("/turtlebot3/polygon/gt", Marker, queue_size=1)
        self.pub_amcl = rospy.Publisher("/turtlebot3/polygon/amcl", Marker, queue_size=1)
        self.pub_error = rospy.Publisher("/error/dist", Float32, queue_size=1)
        self.pub_error_or = rospy.Publisher("/error/orientation", Float32, queue_size=1)

    def gt_pos_callback(self, msg):
        self.gt = msg 

    def amcl_pos_callback(self, msg):
        self.amcl = msg

    def getYaw(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def run(self):
        if self.gt != None and self.amcl != None:

            a = np.array([self.gt.pose.pose.position.x, self.gt.pose.pose.position.y])
            b = np.array([self.amcl.pose.pose.position.x, self.amcl.pose.pose.position.y])

            dist = np.linalg.norm(a-b)

            gt_yaw = self.getYaw(self.gt)
            amcl_yaw = self.getYaw(self.amcl)

            self.gt_arr.append(Point(self.gt.pose.pose.position.x, self.gt.pose.pose.position.y, 0))
            self.amcl_arr.append(Point(self.amcl.pose.pose.position.x, self.amcl.pose.pose.position.y, 0))

            error = Float32()

            error.data = dist
            self.pub_error.publish(error)
            error.data = ( gt_yaw - amcl_yaw )* 180 / math.pi
            self.pub_error_or.publish(error)
            
            self.gt_marker.points = self.gt_arr
            self.amcl_marker.points = self.amcl_arr

            self.pub_gt.publish(self.gt_marker)
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

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.scale.x = 0.02
    marker.scale.y = 0.01
    marker.scale.z = 0.1

    return marker

if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()

