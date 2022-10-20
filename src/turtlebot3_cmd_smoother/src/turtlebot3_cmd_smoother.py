#!/usr/bin/env python3
import rospy

import math
import numpy as np

from geometry_msgs.msg import Twist

class PIDVelCmd():

    def __init__(self):
        self.cmd = None

        self.count = 0
        self.threshhold = 8

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_scan = rospy.Subscriber("/cmd_pre_vel", Twist, self.twist_callback)

    def publish_to_topics(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def twist_callback(self, msg):
        if msg.linear.x == 0 and msg.linear.y == 0:
            self.count = self.count + 1

            if self.count <= self.threshhold:
                self.pub.publish(self.cmd)
            else:
                self.cmd = msg
                self.pub.publish(msg)
        else:
            self.count = 0
            self.cmd = msg
            self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('Topic Visualization')

    node = PIDVelCmd()
    rospy.spin()


