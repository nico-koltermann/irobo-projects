#!/usr/bin/env python3
import rospy
import numpy as np
import math

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Visualization():

    def __init__(self):
        self.gt = None
        self.gt_arr = []
        self.ekf = None
        self.ekf_arr = []
        self.scanmatch = None
        self.scanmatch_arr = []

        self.odom = None

        self.gt_topic = "/ground_truth/state"
        self.ekf_topic = "/odometry/filtered"
        self.scanmatch_topic = "/scanmatch/odom"
        self.odom_topic = "/odom"

        self.ekf_marker = getEmptyMarker()
        self.ekf_marker.color.g = 1.0
        self.gt_marker = getEmptyMarker()
        self.gt_marker.color.r = 1.0
        self.sm_marker = getEmptyMarker()
        self.sm_marker.color.r = 1.0
        self.sm_marker.color.g = 1.0

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_gt_pos = rospy.Subscriber(self.gt_topic, Odometry, self.gt_pos_callback)
        self.sub_ekf_pos = rospy.Subscriber(self.ekf_topic, Odometry, self.sub_ekf_pos)
        self.sub_sm_pos = rospy.Subscriber(self.scanmatch_topic, Odometry, self.sm_callback)

        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

    def publish_to_topics(self):
        self.pub_gt = rospy.Publisher("/turtlebot3/polygon/gt", Marker, queue_size=1)
        self.pub_ekf = rospy.Publisher("/turtlebot3/polygon/ekf", Marker, queue_size=1)
        self.pub_sm = rospy.Publisher("/turtlebot3/polygon/sm", Marker, queue_size=1)

        self.pub_error = rospy.Publisher("/ekf/error/dist", Float32, queue_size=1)
        self.pub_error_or = rospy.Publisher("/ekf/error/orientation", Float32, queue_size=1)  
        self.pub_error_sm = rospy.Publisher("/sm/error/dist", Float32, queue_size=1)
        self.pub_error_or_sm = rospy.Publisher("/sm/error/orientation", Float32, queue_size=1)        
        self.pub_error_od = rospy.Publisher("/odom/error/dist", Float32, queue_size=1)
        self.pub_error_or_od = rospy.Publisher("/odom/error/orientation", Float32, queue_size=1)

    def odom_callback(self, msg):
        self.odom = msg

    def sm_callback(self, msg):
        self.scanmatch = msg

    def gt_pos_callback(self, msg):
        self.gt = msg 

    def sub_ekf_pos(self, msg):
        self.ekf = msg

    def getYaw(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def run(self):
        if self.gt != None and self.ekf != None and self.scanmatch != None:
            a = np.array([self.gt.pose.pose.position.x, self.gt.pose.pose.position.y])
            b = np.array([self.ekf.pose.pose.position.x, self.ekf.pose.pose.position.y])
            c = np.array([self.scanmatch.pose.pose.position.x, self.scanmatch.pose.pose.position.y])
            d = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])

            dist_ab = np.linalg.norm(a-b)
            dist_ac = np.linalg.norm(a-c)
            dist_ad = np.linalg.norm(a-d)

            gt_yaw = self.getYaw(self.gt)
            ekf_yaw = self.getYaw(self.ekf)
            sm_yaw = self.getYaw(self.scanmatch)
            odom_yaw = self.getYaw(self.odom)

            self.gt_arr.append(Point(self.gt.pose.pose.position.x, self.gt.pose.pose.position.y, 0))
            self.ekf_arr.append(Point(self.ekf.pose.pose.position.x, self.ekf.pose.pose.position.y, 0))
            self.scanmatch_arr.append(Point(self.scanmatch.pose.pose.position.x, self.scanmatch.pose.pose.position.y, 0))

            error = Float32()

            error.data = dist_ab
            self.pub_error.publish(error)
            error.data = ( gt_yaw - ekf_yaw ) * 180 / math.pi
            self.pub_error_or.publish(error)

            error.data = dist_ac
            self.pub_error_sm.publish(error)
            error.data = ( gt_yaw - sm_yaw ) * 180 / math.pi
            self.pub_error_sm.publish(error)

            error.data = dist_ad
            self.pub_error_od.publish(error)
            error.data = ( gt_yaw - odom_yaw ) * 180 / math.pi
            self.pub_error_od.publish(error)

            self.gt_marker.points = self.gt_arr
            self.ekf_marker.points = self.ekf_arr
            self.sm_marker.points = self.scanmatch_arr

            self.pub_gt.publish(self.gt_marker)
            self.pub_ekf.publish(self.ekf_marker)
            self.pub_sm.publish(self.sm_marker)


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

    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.1

    return marker

if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()

