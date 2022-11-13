#!/usr/bin/python3
# license removed for brevity
from turtle import pos
import rospy
from std_msgs.msg import String
import math
from copy import deepcopy
from pprint import pprint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from queue import Queue
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv 
import sys
import numpy as np 
from collections import defaultdict 
import time 

class DataLoader:
    def __init__(self, filename):
        self.filename = filename
        self.data = defaultdict(list)
        self.read()
    def read(self):
        with open(self.filename, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            line_count = 0
            max_y = max_x = 0
            for row in csv_reader:
                for key, value in row.items():
                    key = key.strip()
                    val = float(value)
                    if key == 'y':
                        val *= 0.7
                        max_y = max(max_y, val)
                    else:
                        max_x = max(max_x, val)
                    self.data[key].append(val)
                line_count += 1
            print(f'Processed = {line_count} lines max_x = {max_x:.3f} lines max_y = {max_y:.3f}.')

    def __getitem__(self, item):
        return self.data.get(item)

    def __len__(self):
        return len(self.data['x'])

class PathViewer(DataLoader):

    def __init__(self, filename, roomba_name):
        super().__init__(filename)
        topic="%spath"%roomba_name
        self._pub = rospy.Publisher(topic, Marker, queue_size=10)
        colorCount = 0 if roomba_name == '/roomba20/' else 1
        self.color = ColorRGBA(0, 1, colorCount, 1)
        time.sleep(2)
        self.publish()
        

    def getMarker(self):
        marker_ = Marker()
        marker_.header.frame_id = "map"
        marker_.header.stamp = rospy.Time.now()
        marker_.type = marker_.POINTS
        marker_.action = marker_.ADD
        marker_.id = 1

        for x, y in zip(self['x'], self['y']):
            point = Point()
            point.x = x
            point.y = y
            point.z = 0
            marker_.points.append(point)
        marker_.scale.x = 0.02
        marker_.scale.y = 0.02
        marker_.scale.z = 0.02
        
        marker_.color = self.color
        
        return marker_

    def publish(self):
        marker = self.getMarker()
        self._pub.publish(marker)

class TrajectoryGenerator(PathViewer):
    def __init__(self, filename, roomba_name):
        super().__init__(filename, roomba_name)
        self.roomba_name = roomba_name
        self.__trajCount = 0
        self.pub_viz_state =  rospy.Publisher('%spath'%roomba_name, Marker, queue_size=10)
        self.pub_cntrl_goal = rospy.Publisher('%smove_base_simple/goal' % roomba_name, PoseStamped, queue_size=10)
        # self.loop = rospy.Timer(rospy.Duration(0.5), self.timerCallback)
        self.loop = rospy.Timer(rospy.Duration(3.0), self.timerCallback)


    def setPoint(self, point):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = '%ssetpoint'%self.roomba_name
        marker.id = 201
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # set up scale of the points
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color = self.color
        

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.05
        return marker
    
    def timerCallback(self, event):

        if self.__trajCount >= len(self):
            return 
        
        i = self.__trajCount
        coord = [self['x'][i], self['y'][i]]
        print(f'{self.roomba_name} current setpoint = {coord}')
        # visualize sphere as next goal location 
        self.pub_viz_state.publish(self.setPoint(coord))
        
        pose = PoseStamped()
        pose.header.frame_id = "last" if len(self) == self.__trajCount else "intermediate"
        pose.pose.position.x = coord[0]
        pose.pose.position.y = coord[1]
        # send this goal to the controller to generate velocities 
        self.pub_cntrl_goal.publish(pose)
        self.__trajCount += 1 


if __name__ == '__main__':

    rospy.init_node('roomba_traj_viewer', anonymous=True)
    csv_file = sys.argv[1]

    # csv_file = "/home/robo/catkin_roomba/src/BoVo/results/robo0.csv"
    # csv_file = "/home/robo/catkin_roomba/src/MDP_BOVO_data/Alice_plan_refine.csv"
    roomba_name=rospy.get_namespace()
    node_name = rospy.get_name().split('/')[-1]
    print("seraching param = ", f'{roomba_name}{node_name}/delay')
    delay = rospy.get_param(f'{roomba_name}{node_name}/delay', 0.0)
    time.sleep(delay)
    print("roomba_name:", roomba_name, " | delay: ", delay)
    rospy.loginfo("roomba traj controller started")
    trajGen = TrajectoryGenerator(filename=csv_file, roomba_name=roomba_name )
    rospy.spin()
    
