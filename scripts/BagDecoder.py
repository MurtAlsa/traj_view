#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Point, PoseStamped, TwistWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import tf
import math
from copy import deepcopy

class VizRobot:
    def __init__(self, state_topic, viz_pub_topic, robotID):
        self._state_sub = rospy.Subscriber(state_topic, Odometry, self.state_callback)
        self._path_pub = rospy.Publisher(viz_pub_topic, Marker, queue_size=10)
        self.robotID = robotID
        self.__msgCount = 0
        self.__poseTracker = []
        self.__initialized = False

    def fixOrientation(self, quat, correction):
        eulers = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        np_quat = quaternion_from_euler(0, 0, eulers[2] + correction)
        geom_quat = deepcopy(quat)
        geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w = np_quat
        return geom_quat

    def complimentaryFilter(self, state):
        if not self.__initialized:
            self.__state = state
        alpha = 0.99
        self.__state[0] = self.__state[0] * alpha + (1 - alpha) * state[0]
        self.__state[1] = self.__state[1] * alpha + (1 - alpha) * state[1]
        return self.__state.copy()

    def state_callback(self, msg:Odometry):
        # rospy.loginfo(msg)
        state = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        state = self.complimentaryFilter(state)

        quat = self.fixOrientation(msg.pose.pose.orientation, math.pi/2)
        marker = self.current_state_pub(state, quat)
        self._path_pub.publish(marker)

        self.__poseTracker.append(state)
        trajmarker = self.prev_traj_pub()
        self._path_pub.publish(trajmarker)

        self.__initialized = True


    def current_state_pub(self, state, quat):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'Roomba/state'
        marker.id = self.robotID
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://traj_view/config/ROOMBA.dae"
        # set up scale of the points
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.50
        marker.color.r = 1
        marker.color.g = 1 - (self.robotID - 20)
        marker.color.b = 0

        marker.pose.position.x = state[0]
        marker.pose.position.y = state[1]
        marker.pose.position.z = 0.0

        marker.pose.orientation = quat


        return marker

    def prev_traj_pub(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'Roomba/traj'
        marker.id = self.robotID
        marker.type = Marker.POINTS
        marker.action = Marker.ADD


        # set up scale of the points
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.a = 0.50
        marker.color.r = 1
        marker.color.g = 1 - (self.robotID - 20)
        marker.color.b = 0

        for state in self.__poseTracker:
            p = Point()
            p.x = state[0]
            p.y = state[1]
            p.z = 0
            marker.points.append(p)
        return marker

if __name__ == "__main__":
    rospy.init_node("BagDecoder", anonymous=True)
    rospy.loginfo("Bag Decoder Initiated")
    roomba20 = VizRobot("/roomba20/odometry/filtered", "/roomba/state", 20)
    roomba20 = VizRobot("/roomba21/odometry/filtered", "/roomba/state", 21)

    rospy.spin()