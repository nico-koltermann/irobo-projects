#!/usr/bin/env python3
import rospy
import numpy as np
import math

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Visualization():

    def __init__(self):
        self.gt = None
        self.gt_poly = PolygonStamped()
        self.ekf = None
        self.ekf_poly = PolygonStamped()

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_gt_pos = rospy.Subscriber("/odometry/filtered", Odometry, self.gt_pos_callback)
        self.sub_ekf_pos = rospy.Subscriber("/odom", Odometry, self.sub_ekf_pos)

    def publish_to_topics(self):
        self.pub_gt = rospy.Publisher("/turtlebot3/polygon/gt", PolygonStamped, queue_size=1)
        self.pub_ekf = rospy.Publisher("/turtlebot3/polygon/ekf", PolygonStamped, queue_size=1)

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
        if self.gt != None and self.ekf != None:
            a = np.array([self.gt.pose.pose.position.x, self.gt.pose.pose.position.y])
            b = np.array([self.ekf.pose.pose.position.x, self.ekf.pose.pose.position.y])

            dist = np.linalg.norm(a-b)

            gt_yaw = self.getYaw(self.gt)
            ekf_yaw = self.getYaw(self.ekf)

            self.gt_poly.header = self.gt.header
            self.ekf_poly.header = self.gt.header

            self.gt_poly.polygon.points.append(Point(self.gt.pose.pose.position.x, self.gt.pose.pose.position.y, 0))
            self.ekf_poly.polygon.points.append(Point(self.ekf.pose.pose.position.x, self.ekf.pose.pose.position.y, 0))

            self.pub_gt.publish(self.gt_poly)
            self.pub_ekf.publish(self.ekf_poly)

            rospy.loginfo("Error of ekf: %s", str(dist))
            rospy.loginfo("Error of orientation: %s", str((gt_yaw - ekf_yaw )* 180 / math.pi))


if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = Visualization()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        node.run()
        r.sleep()

