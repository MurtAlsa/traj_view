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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv 
import sys
q = Queue()


def dist(x, y):
    dx = x[0] - y[0]
    dy = x[1] - y[1]
    return math.sqrt(dx**2 + dy**2)

def interpolate(x, y, theta):
    nx = x[0] * theta + (1 - theta) * y[0]
    ny = x[1] * theta + (1 - theta) * y[1]
    return[nx, ny]

def refine_path(target):
    assert len(target) > 1, "path must have two points at least"
    path = deepcopy(target)
    result = []
    delta = dist(target[0], target[1]) / 2
    # print(f'resolution {delta:.3f}')
    while len(path) > 1:
        x = path.pop()
        y = path[-1]
        result.append(x)
        if dist(x, y) > delta:
            new_point = interpolate(x, y, 0.5)
            path.append(new_point)
    result.append(path.pop())
    return result[::-1]

def traj_marker(path):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = '%spath'%roomba_name
    marker.id = 20
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # set up scale of the points
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    for p in path:
        gp = Point()
        gp.x = p[0]
        gp.y = p[1]
        gp.z = 0.2
        marker.points.append(gp)
    return marker

def roomba_setpoint(point):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = '%ssetpoint'%roomba_name
    marker.id = 201
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    # set up scale of the points
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.2
    return marker

traj = []

def ekf_odometry_callback(msg):

    pose = msg.pose.pose
    x,y,z = pose.position.x, pose.position.y, pose.position.z
    angels = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    print("angels",angels)
    state = [x,y,angels[2]]
    traj.append(state)


def publish_path():
    # path = [[0, 0], [2 , 0], [2, -1], [0, -1], [0, -2], [2, -2],[2, -3], [0,-3], [0, 0]]
    # path = [[0, 0], [1.5 , 0], [1.5, -1], [0, -1], [0, -2], [1.5, -2], [0,-2], [0, 0]]
    # path = [[0, 0], [1.5 , 0], [1.5, 1], [0, 1], [0, 2], [1.5, 2], [0, 2], [0, 0]]
    # path = [[0, 0], [0 , 1.5], [1, 1.5], [1, 0], [1.8, 0], [1.8, 1.5], [1.8, 0], [0, 0]] # start 0, 0

    # path = [[1.8, 0], [1.8, 1.5], [1.0, 1.5], [1.0, 0], [0, 0]]
    path = [[1.8, 0], [1.8, 1.4], [1.0, 1.4], [1.0, 0], [0, 0]]
    result = refine_path(path) # 0.5 resolution
    result = refine_path(result) # 0.25 resolution
    result = refine_path(result) # 0.125 resolution
    result = refine_path(result) # 0.062 resolution
    # pub.publish(traj_marker(result))
    for _ in range(2):
        pub.publish(traj_marker(result))
        rospy.sleep(1)
        print('publishing traj')

    for p in result:
        q.put(p)

def update_setpoints(event):
    if not q.empty():
        setpoint = q.get()
        pub.publish(roomba_setpoint(setpoint))
        
        pose = PoseStamped()
        size = q.qsize()
        pose.header.frame_id = "last"
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        goal_pub.publish(pose)

if __name__ == '__main__':

    rospy.init_node('roomba_traj_viewer', anonymous=True)
    roomba_name=rospy.get_namespace()
    print("roomba_name:", roomba_name)
    pub = rospy.Publisher('%spath'%roomba_name, Marker, queue_size=10)
    goal_pub = rospy.Publisher('%smove_base_simple/goal' % roomba_name, PoseStamped, queue_size=10)
    rospy.loginfo("roomba traj controller started")
    publish_path()
    rospy.Subscriber("%sodometry/filtered"%roomba_name, Odometry, ekf_odometry_callback) 
    rospy.Timer(rospy.Duration(0.50), update_setpoints)
    rospy.spin()
    
