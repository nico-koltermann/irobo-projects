#!/usr/bin/env python3
import rospy
import numpy as np
import math

from numpy import asarray
from numpy import savetxt

from random import random

import sys
import time

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from visualization_msgs.msg import Marker, MarkerArray

import markov_calculation as markovSolver

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

class MarkovDecision():

    def __init__(self):
        self.markerArray = MarkerArray()
        self.gridMap = None

        self.markov = None

        self.finalMap = None
        self.actionMap = None
        self.visit = None

        self.goal_x = 32    
        self.goal_y = 39

        self.avoid_x = 28
        self.avoid_y = 34

        self.offset = -10
        self.cell_size = 0.3

        self.gamma = 2.0

        self.iteration = 30

        self.path = None
        self.foundGoal = False

        self.robot_pose = None
        self.robo_x = None
        self.robo_y = None

        self.actions = {
            'u': (0,1), 
            'r': (1,0),
            'd': (0,-1),
            'l': (-1,0)
        }

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
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    def  odom_callback(self, msg):
        self.robot_pose = msg

    def  map_callback(self, msg):
        
        self.gridMap = msg

        finalMap = []
        otherArray = []
        for x in range(0, msg.info.width):
            row = []
            for y in range(0, msg.info.height):
                dataPoint = msg.data[y * msg.info.width + x] 
                if (dataPoint == -1 or dataPoint == 100):
                    row.append(-1)
                else:
                    row.append(0)

            finalMap.append(np.array(row))

        self.finalMap = np.array(finalMap)

        self.calculateMarkov()
   
    def calculateMarkov(self):

        self.markov = markovSolver.Markov()

        self.markov.setAvoid(self.avoid_x, self.avoid_y)
        self.markov.setGoal(self.goal_x, self.goal_y)

        self.markov.setWorld(self.finalMap)

        self.markov.init()

        for _ in range(0, self.iteration):
            self.makeIteration()
            time.sleep(0.1)

        ma = self.markov.getWorldMap()
        ac = self.markov.getActionMap()
        self.saveMap(ma)
        self.saveActionMap(ac)

        if self.foundGoal:
            self.move_robot()
    
    def move_robot(self):
        speed = Twist()
        self.path.pop(0)
        i = 1
        for p in self.path:
            goal_reached = False
            while(not goal_reached):
                inc_x = p.x - self.robot_pose.pose.pose.position.x
                inc_y = p.y - self.robot_pose.pose.pose.position.y

                dist = math.dist([p.x, p.y], [self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y])

                if dist < 0.05:
                    print("#######################")
                    print("Goal " + str(i) + " / " + str(len(self.path)) + " reached!")
                    print("#######################")
                    i = i + 1
                    goal_reached = True
                    break

                rot_q = self.robot_pose.pose.pose.orientation
                (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

                angle_to_goal = atan2(inc_y, inc_x)

                if angle_to_goal - theta > 0.2:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.5
                elif angle_to_goal - theta < -0.2:
                    speed.linear.x = 0.0
                    speed.angular.z = -0.5
                else:
                    speed.linear.x = 0.26
                    speed.angular.z = 0.0

                self.pub_cmd.publish(speed)
                time.sleep(0.005)
        
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub_cmd.publish(speed)
        rospy.loginfo("###########################")
        rospy.loginfo("FINISHED")
        rospy.loginfo("###########################")


    def makeIteration(self, iteration=1):
        self.markov.doRewardIteration(iteration)
        self.finalMap = self.markov.getWorldMap()
        self.actionMap = self.markov.getActionMap()
        self.createMarker()

    def saveMap(self, world):
        np.savetxt('/home/nico/IST_tecnico/irobo-projects/src/turtlebot3_markov_decision/src/dataMap.dat', np.around(world, decimals=2),fmt='%.2f',delimiter='\t')

    def saveActionMap(self, world):
        np.savetxt('/home/nico/IST_tecnico/irobo-projects/src/turtlebot3_markov_decision/src/dataActions.dat',world,fmt='%s',delimiter='\t')

    def getMapData(self, x, y):
        return self.gridMap.data[y * self.gridMap.info.width + x]

    #####################################################################
    ##############          Visualization                   #############
    #####################################################################

    def getBestDirection(self, x, y):
        above   = self.finalMap[x][y+1]
        right   = self.finalMap[x+1][y] 
        below   = self.finalMap[x][y-1] 
        left    = self.finalMap[x-1][y]
        
        directions = [above, right, below, left]
        ind = np.argmax(np.array(directions))
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

        closest = float('inf')
        robot_point = None
        for x in range(0, self.gridMap.info.width):
            for y in range(0, self.gridMap.info.height):

                if  self.gridMap.data[y * self.gridMap.info.width + x] == 0:
                    x_coord = (x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    y_coord = (y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)

                    dist = math.dist([x_coord, y_coord], [self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y])

                    if dist < closest:
                        robot_point = (x, y)
                        closest = dist

        path = self.getEmptyMarker(4)

        self.foundGoal = False

        max_iter = 100
        count = 0
        next_x = robot_point[0]
        next_y = robot_point[1]
        while(count < max_iter):

            nextAction = self.actionMap[next_x][next_y]
            if nextAction == 'g':
                p = Point(
                    (next_x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8), \
                    (next_y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8), 0.05)
                path.points.append(p)
                self.foundGoal = True
                break

            p = Point(
                (next_x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8), \
                (next_y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8), 0.05)

            path.points.append(p)

            action = self.actions[nextAction]
            next_x = next_x + action[0]
            next_y = next_y + action[1]
            count = count + 1

        self.path = path.points

        path.color.r = 0.0
        path.color.g = 0.0
        path.color.b = 0.8
        path.color.a = 0.8

        path.scale.x = 0.05
        path.scale.y = 0.01
        path.scale.z = 0.1

        path.id = i 
        i = i + 1

        self.markerArray.markers.append(path)

        lower_border = 100
        for x in range(0, self.finalMap.shape[0]):
                for y in range(0, self.finalMap.shape[1]):
                    val = self.finalMap[x][y]
                    if val > 0:
                        if lower_border > val:
                            lower_border = val

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
                    elif x == robot_point[0] and y == robot_point[1]:
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                        specialPoint = True
                    elif x == self.avoid_x and y == self.avoid_y:
                        marker.color.r = 0.8
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        specialPoint = True
                    elif dataPoint == 0 and not specialPoint:
                         # Set the color
                        marker.color.r = 0.0 
                        marker.color.g =  (self.finalMap[x][y] - lower_border) / (np.max(self.finalMap) - lower_border)
                        marker.color.b = 0.0
                    else: 
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                    marker.color.a = 1.0

                    marker.pose.position.x = (x * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    marker.pose.position.y = (y * self.gridMap.info.resolution) + self.offset + (self.cell_size * 0.8)
                    marker.pose.position.z = 0

                    self.markerArray.markers.append(marker)
                    if not specialPoint:
                        self.markerArray.markers.append(arrow)

        self.pub_viz.publish(self.markerArray)

#####################################################################
##############              Node                        #############
#####################################################################

if __name__ == "__main__":
    rospy.init_node('MarkovDecision')

    node = MarkovDecision()

    rospy.spin()

