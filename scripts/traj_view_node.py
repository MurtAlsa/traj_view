#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import math
from copy import deepcopy
from pprint import pprint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from queue import Queue

q = Queue()


def dist(x, y):
    dx = x[0] - y[0]
    dy = x[1] - y[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def interpolate(x, y, theta):
    nx = x[0] * theta + (1 - theta) * y[0]
    ny = x[1] * theta + (1 - theta) * y[1]
    return [nx, ny]


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
    marker.ns = '/roomba20/path'
    marker.id = 20
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # set up scale of the points
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker.color.a = 0.30
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    for p in path:
        gp = Point()
        gp.x = p[0]
        gp.y = p[1]
        gp.z = 0
        marker.points.append(gp)
    return marker


def roomba_setpoint(point):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = '/roomba20/setpoint'
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
    marker.pose.position.z = 0.0
    return marker


def publish_path():
    default_path = [[0, 0], [2, 0], [2, -1], [0, -1], [0, -2], [2, -2], [2, -3], [0, -3], [0, 0]]
    path = rospy.get_param('/%s/path'% rospy.get_name(), default_path)
    iteration = rospy.get_param('/%s/iteration' % rospy.get_name(), 4)
    for _ in range(iteration):
        path = refine_path(path)
    for _ in range(2):
        pub.publish(traj_marker(path))
        rospy.sleep(1)
        print('publishing traj')

    for p in path:
        q.put(p)


def update_setpoints(event):
    if not q.empty():
        setpoint = q.get()
        pub.publish(roomba_setpoint(setpoint))

        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        goal_pub.publish(pose)
        rospy.loginfo(f"current setpoint {setpoint}")


if __name__ == '__main__':
    rospy.init_node('roomba_traj_viewer', anonymous=True)
    pub = rospy.Publisher('/roomba20/path', Marker, queue_size=10)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.loginfo("roomba traj controller started")
    loop_time = iteration = rospy.get_param('/trajViewer/loop_time', 0.5)
    publish_path()
    rospy.Timer(rospy.Duration(loop_time), update_setpoints)
    rospy.spin()
