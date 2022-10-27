#!/usr/bin/env python3
import rospy
import numpy as np
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, GridCells

class MarkovDecision():

    def __init__(self):
        self.cells = None

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def publish_to_topics(self):
        self.pub_cells = rospy.Publisher("/viz/cell", GridCells, queue_size=1)

    def  map_callback(self, msg):
        
        offset = -9.75
        cell_size = 0.3

        self.cells = GridCells()
        self.cells.header.stamp = rospy.Time().now()
        self.cells.header.frame_id = "map"

        self.cells.cell_height = cell_size
        self.cells.cell_width = cell_size

        
        for width in range(0, msg.info.width):
            for height in range(0, msg.info.height):
                if 0 <= msg.data[height * msg.info.width + width] < 10:
                    p = Point()
                    p.x = (width * msg.info.resolution + offset)
                    p.y = (height * msg.info.resolution + offset)
                    self.cells.cells.append(p)

        rospy.logwarn(len(self.cells.cells))

        self.pub_cells.publish(self.cells)

if __name__ == "__main__":
    rospy.init_node('MarkovDecision')

    node = MarkovDecision()

    rospy.spin()

