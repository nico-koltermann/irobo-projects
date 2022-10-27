#!/usr/bin/env python3
import rospy
import numpy as np
import math

from random import random

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, GridCells
from visualization_msgs.msg import Marker, MarkerArray

class MarkovDecision():

    def __init__(self):
        self.cells = None

        self.finalArray = MarkerArray()
        self.mapPoints = []
        self.gridMap = None

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def publish_to_topics(self):
        self.pub_cells = rospy.Publisher("/viz/cell", GridCells, queue_size=1)
        self.pub_viz = rospy.Publisher("/viz/test", MarkerArray, queue_size=1)

    def getEmptyMarker(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1

        # Set the scale of the marker
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.01

        # Set the pose of the marker
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def  map_callback(self, msg):
        
        self.gridMap = msg

        offset = -10
        cell_size = 0.3

        self.cells = GridCells()
        self.cells.header.stamp = rospy.Time().now()
        self.cells.header.frame_id = "map"

        self.cells.cell_height = cell_size
        self.cells.cell_width = cell_size

        i = 0
        for x in range(0, msg.info.width):
            for y in range(0, msg.info.height):
                dataPoint = msg.data[y * msg.info.width + x] 
                if dataPoint == 100 or dataPoint == 0:
                    marker = self.getEmptyMarker()

                    marker.id = i
                    i = i + 1

                    # Set the color
                    marker.color.r = 1.0 * random()
                    marker.color.g = 1.0 * random()
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                    marker.pose.position.x = (x * msg.info.resolution) + offset + (cell_size * 0.8)
                    marker.pose.position.y = (y * msg.info.resolution) + offset + (cell_size * 0.8)
                    marker.pose.position.z = 0

                    self.finalArray.markers.append(marker)

                    self.mapPoints.append( Point( 
                        ((x * msg.info.resolution) + offset) + (cell_size * 0.8),
                        ((y * msg.info.resolution) + offset) + (cell_size * 0.8),
                        0
                    ))

        self.pub_viz.publish(self.finalArray)

        self.calculateMarkov()


    def calculateMarkov(self):
        pass

if __name__ == "__main__":
    rospy.init_node('MarkovDecision')

    node = MarkovDecision()

    rospy.spin()

