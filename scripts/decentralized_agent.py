#!/usr/bin/python3
# https://github.com/MengGuo/RVO_Py_MAS/blob/master/example.py
from RVO import RVO_update, reach, compute_V_des, reach, distance
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from time import sleep
import numpy as np
from threading import Thread
from functools import partial


class DecentralizedAgent(Thread):
    # ------------------------------
    # define workspace model
    ws_model = dict()
    # circular obstacles, format [x,y,rad]
    # no obstacles
    ws_model['circular_obstacles'] = []
    # with obstacles
    # ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
    # rectangular boundary, format [x,y,width/2,heigth/2]
    ws_model['boundary'] = []
    # discrete time
    dt = 0.5

    def __init__(self):
        super().__init__()


        name = rospy.get_namespace()

        print(name)
        goal_topic = rospy.get_param("/%s/%s/goalTopic" % (name, name))
        robot_name = rospy.get_param('/%s/%s/robotName' % (name, name))
        other_robot_name = rospy.get_param('/%s/%s/otherRobotName' % (name, name))
        all_robots = [robot_name] + other_robot_name
        self.ws_model['robot_radius'] = rospy.get_param("/%s/%s/radius" % (name, name), 0.345)

        self._goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        self.V = [[0, 0] for _ in range(len(all_robots))]
        self._measurements = {}
        self.ego_robot = robot_name
        for _name in all_robots:
            callback = partial(self.state_callback, _name)
            self._state_sub = rospy.Subscriber('/%s/%s/filtered/odometry' % (_name, _name), Odometry, callback)

    @staticmethod
    def publish_setpoint(setpoint, pub):
        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        pub.publish(pose)

    def state_callback(self, robot_name: str, msg: Odometry):
        pose = msg.pose.pose
        eulers = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        state = np.array([0] * 4)
        state[0] = pose.position.x
        state[1] = pose.position.y
        # state[2] = eulers[-1]
        state[2] = msg.twist.twist.linear.x
        state[3] = msg.twist.twist.linear.y
        self._measurements[robot_name] = state

    def compute_control(self):

        X = [z[:2].tolist() for z in self._measurements.values()]
        # V = [z[2:].tolist() for z in self._measurements.values()]
        V = self.V
        goal = [self._goal_position.tolist()] + [self._measurements[key].tolist() for key in self._measurements if
                                                 str(key) != self.ego_robot]
        V_max = [1.0 for _ in range(len(X))]
        # compute desired vel to goal
        V_des = compute_V_des(X, goal, V_max)
        # compute the optimal vel to avoid collision
        V = RVO_update(X, V_des, V, self.ws_model)

        for i in range(len(X)):
            X[i][0] += V[i][0] * self.dt
            X[i][1] += V[i][1] * self.dt

        self.V = V
        return np.array(X[0])

    def run(self) -> None:
        name = rospy.get_namespace()

        rate = rospy.Rate(self.dt)
        path = rospy.get_param("/%s/%s/path" % (name, name))
        rospy.loginfo(f"{name} is running {path}")
        self._goal_position = np.array(path[0])

        while not rospy.is_shutdown():
            if path:
                # compute distance
                robot = self._measurements[self.ego_robot][:2]
                dist = distance(self._goal_position.tolist(), robot.tolist())
                if dist >= self.ws_model['robot_radius']:

                    u = self.compute_control()
                    next_state = robot + u
                    rospy.loginfo(f"{rospy.get_name()} dist = {dist}")
                    self.publish_setpoint(next_state, self._goal_pub)
                else:
                    self._goal_position = np.array(path.pop(0))

            rate.sleep()

        rospy.loginfo("Thread finished")
        sleep(5)


if __name__ == '__main__':
    rospy.init_node('decentralized_robot', anonymous=True)

    rospy.loginfo(f"decentralized robot {rospy.get_name()} initialized")
    robot = DecentralizedAgent()
    sleep(5)
    robot.start()
    rospy.spin()
