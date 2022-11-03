#!/usr/bin/env python3
import rospy
import numpy as np
import math

from numpy import asarray
from numpy import savetxt

from random import random

import sys

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from visualization_msgs.msg import Marker, MarkerArray

class MarkovDecision():

    def __init__(self):
        self.finalArray = MarkerArray()
        self.gridMap = None

        self.finalMap = None
        self.visit = None

        self.goal_x = 32    
        self.goal_y = 39

        self.avoid_x = 28
        self.avoid_y = 34

        self.offset = -10
        self.cell_size = 0.3

        self.wall_cost = 20
        self.avoid_cost = 500
        self.goal_cost = -50

        self.gamma = 2.0

        self.iteration = 3

        self.robot_pose = None

        self.load_parameters()
        self.subscribe_to_topics()
        self.publish_to_topics()

    def load_parameters(self):
        pass

    def subscribe_to_topics(self):
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.sub_pose = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def publish_to_topics(self):
        self.pub_viz = rospy.Publisher("/viz/test", MarkerArray, queue_size=1)

    def  odom_callback(self, msg):
        self.robot_pose = msg

    def  map_callback(self, msg):
        
        self.gridMap = msg

        i = 0

        finalMap = []
        otherArray = []
        for x in range(0, msg.info.width):
            row = []
            for y in range(0, msg.info.height):
                dataPoint = msg.data[y * msg.info.width + x] 

                if (dataPoint == -1 or dataPoint == 100):
                    row.append(-1)
                else:
                    if x == self.avoid_x and y == self.avoid_y:
                        row.append(self.avoid_cost)
                    elif x == self.goal_x and y == self.goal_y:
                        row.append(self.goal_cost)
                    else:
                        row.append(dataPoint)


            finalMap.append(np.array(row))

        self.finalMap = np.array(finalMap)

        self.calculateMarkov()
   
    def calculateMarkov(self):

        self.visit = np.full((self.finalMap.shape[1], self.finalMap.shape[0]), False, dtype=bool)

        self.iterateFirst()

        for _ in range(0, self.iteration):
            self.iterateN()

        self.createMarker()

        rospy.logwarn("Finished calculation MPD")

        self.saveMap()

    def saveMap(self):
        np.savetxt('/home/nico/IST_tecnico/irobo-projects/src/turtlebot3_markov_decision/src/data.dat', np.around(self.finalMap, decimals=2),fmt='%.2f',delimiter='\t')

    def getNeighbors(self, x, y): 
        above   = (1 if self.getMapData(x, y+1) == 0.0 else self.wall_cost) 
        below   = (1 if self.getMapData(x, y-1) == 0.0 else self.wall_cost) 
        left    = (1 if self.getMapData(x-1, y) == 0.0 else self.wall_cost) 
        right   = (1 if self.getMapData(x+1, y) == 0.0 else self.wall_cost) 
        return above, below, left, right 

    def getDiagonalNeighbors(self, x, y): 
        aboveR   = (1 if self.getMapData(x+1, y+1) == 0.0 else self.wall_cost) 
        aboveL   = (1 if self.getMapData(x-1, y+1) == 0.0 else self.wall_cost) 
        belowR   = (1 if self.getMapData(x+1, y-1) == 0.0 else self.wall_cost) 
        belowL    = (1 if self.getMapData(x-1, y-1) == 0.0 else self.wall_cost) 
        return aboveR, aboveL, belowR, belowL 

    def getNeighborsCost(self, x, y): 
        above   = self.finalMap[x][y+1]
        below   = self.finalMap[x][y-1] 
        left    = self.finalMap[x-1][y]
        right   = self.finalMap[x+1][y] 
        return above, below, left, right 

    def getDiagonalNeighborsCost(self, x, y): 
        aboveR   = self.finalMap[x+1][y+1]
        aboveL   = self.finalMap[x-1][y+1]
        belowR   = self.finalMap[x+1][y-1]
        belowL   = self.finalMap[x-1][y-1]
        return aboveR, aboveL, belowR, belowL 

    def getMapData(self, x, y):
        return self.gridMap.data[y * self.gridMap.info.width + x]

    def iterateFirst(self):
        for i in range(0, self.finalMap.shape[0]): 
            for j in range(0, self.finalMap.shape[1]):
                currentCell = float(self.getMapData(i, j))

                if not currentCell == -1.0:
                    above, below, left, right = self.getNeighbors(i, j)
                    # aboveR, aboveL, belowR, belowL  = self.getDiagonalNeighbors(i, j)

                    addToNeighbors = above + below + left + right + 1
                    # addDiagonalNeighbors = aboveR + aboveL + belowR + belowL

                    self.finalMap[i][j] = self.finalMap[i][j] + addToNeighbors 


    def iterateN(self):
        nextMap = self.finalMap.copy()

        for i in range(0, self.finalMap.shape[0]): 
            for j in range(0, self.finalMap.shape[1]):
                currentCell = float(self.getMapData(i, j))

                if not currentCell == -1.0:
                    above, below, left, right = self.getNeighborsCost(i, j)
                    aboveR, aboveL, belowR, belowL  = self.getDiagonalNeighborsCost(i, j)

                    addToNeighbors = above + below + left + right + 1
                    addDiagonalNeighbors = aboveR + aboveL + belowR + belowL

                    nextMap[i][j] = self.finalMap[i][j] + addToNeighbors + addDiagonalNeighbors

        for i in range(self.finalMap.shape[0], 0): 
            for j in range(self.finalMap.shape[1], 0):
                currentCell = float(self.getMapData(i, j), 0)

                if not currentCell == -1.0:
                    above, below, left, right = self.getNeighborsCost(i, j)
                    aboveR, aboveL, belowR, belowL  = self.getDiagonalNeighborsCost(i, j)

                    addToNeighbors = above + below + left + right + 1
                    addDiagonalNeighbors = aboveR + aboveL + belowR + belowL

                    nextMap[i][j] = self.finalMap[i][j] + addToNeighbors + addDiagonalNeighbors

        self.finalMap = self.finalMap + (self.gamma * nextMap)


    #####################################################################
    ##############          Visualization                   #############
    #####################################################################


    def getBestDirection(self, x, y):
        above   = self.finalMap[x][y+1]
        right   = self.finalMap[x+1][y] 
        below   = self.finalMap[x][y-1] 
        left    = self.finalMap[x-1][y]
        
        directions = [above, right, below, left]
        ind = np.argmin(np.array(directions))
        directionsPoints = [Point(0,0.2,0), Point(0.2,0,0), Point(0,-0.2,0), Point(-0.2,0,0)]

        return directionsPoints[ind]

    def getEmptyMarker(self, marker_type):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = marker_type

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

    def createMarker(self): 
        i = 0
        for x in range(0, self.gridMap.info.width):
            for y in range(0, self.gridMap.info.height):
                dataPoint = self.gridMap.data[y * self.gridMap.info.width + x] 

                if dataPoint == 0:
                    marker = self.getEmptyMarker(1)
                    arrow = self.getEmptyMarker(0)

                    direction = self.getBestDirection(x, y)

                    start = Point()
                    start.x = (x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    start.y = (y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    start.z = 0.05

                    goal = Point()
                    goal.x = (x * self.gridMap.info.resolution) + self.offset + direction.x + (self.cell_size * 0.8)
                    goal.y = (y * self.gridMap.info.resolution) + self.offset + direction.y + (self.cell_size * 0.8)
                    goal.z = 0.05

                    arrow.points.append(start)
                    arrow.points.append(goal)

                    arrow.color.r = 0.0
                    arrow.color.g = 0.0
                    arrow.color.b = 1.0
                    arrow.color.a = 1.0

                    arrow.scale.x = 0.01
                    arrow.scale.y = 0.05
                    arrow.scale.z = 0.1

                    arrow.id = i
                    i = i + 1
                    marker.id = i
                    i = i + 1

                    specialPoint = False

                    if x == self.goal_x and y == self.goal_y:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        specialPoint = True
                    if x == self.avoid_x and y == self.avoid_y:
                        marker.color.r = 0.8
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        specialPoint = True
                    elif dataPoint == 0 and not specialPoint:
                         # Set the color
                        marker.color.r = 0.0 
                        marker.color.g =  1.0 - (self.finalMap[x][y] / np.max(self.finalMap))
                        marker.color.b = 0.0
                    elif dataPoint == 100: 
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                    marker.color.a = 1.0

                    marker.pose.position.x = (x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    marker.pose.position.y = (y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    marker.pose.position.z = 0

                    self.finalArray.markers.append(marker)
                    if not specialPoint:
                        self.finalArray.markers.append(arrow)

        self.pub_viz.publish(self.finalArray)


    def step(self, x, y):

        if not 0 <= y < self.finalMap.shape[0]:
            return
        if not 0 <= x < self.finalMap.shape[1]:
            return

        if self.visit[x][y]:
            return

        currentCell = self.getMapData(x, y)

        if currentCell == 0:
            above, below, left, left, right = self.getNeighbors(x, y)
            
            addToNeighbors = above + below + left + right + currentCell

            self.finalMap[x+1][y] = self.finalMap[x+1][y] + addToNeighbors
            self.finalMap[x-1][y] = self.finalMap[x-1][y] + addToNeighbors
            self.finalMap[x][y+1] = self.finalMap[x][y+1] + addToNeighbors
            self.finalMap[x][y-1] = self.finalMap[x][y-1] + addToNeighbors


        self.visit[x][y] = True

        self.step(x-1, y)
        self.step(x+1, y)
        self.step(x, y-1)
        self.step(x, y+1)

#####################################################################
##############              Node                        #############
#####################################################################

if __name__ == "__main__":
    rospy.init_node('MarkovDecision')

    node = MarkovDecision()

    rospy.spin()

