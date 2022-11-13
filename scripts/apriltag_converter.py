#!/usr/bin/python3
import rospy 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep, time 
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from functools import partial
import re 
import tf 


def odom_to_state(poseMsg:Odometry):
    x = poseMsg.pose.pose.position.x
    y = poseMsg.pose.pose.position.y
    q = [poseMsg.pose.pose.orientation.x, poseMsg.pose.pose.orientation.y, poseMsg.pose.pose.orientation.z, poseMsg.pose.pose.orientation.w]
    state = euler_from_quaternion(q)
    return np.array([x, y, state[-1]])

class OdomFix:
    def __init__(self, name) -> None:
        self._odom_sub = rospy.Subscriber(f"/{name}/odom", Odometry, self.callback_odom)
        self._odom_pub = rospy.Publisher(f"/{name}/odom/fix", Odometry, queue_size=10)
        self._ekf_sub = rospy.Subscriber(f"/{name}/odometry/filtered", Odometry, self.callback_ekf)
        self.initialized = False
        self.odom_initialized = False 
        self.name = name  
        self._wait_time = 100
        self._offsetState = np.array([0, 0, 0])
        self.odom_drift = np.array([0, 0, 0]) 
    
    def callback_odom(self, msg:Odometry):
        # don't process before initialization 
        if not self.initialized: return 

        odomState = odom_to_state(msg)
        odomState[0], odomState[1] = odomState[1], odomState[0] 
        odomState[0] *= -1
        odomState[2] += np.pi/2

        if not self.odom_initialized: 
            self.odom_init = odomState.copy()
            self.odom_initialized = True 

        # update estimate state 
        offset = self.initial_pos - self.odom_init
        # compensate odom drift from ekf state estimator callback
        odomState += offset + self.odom_drift
        # odomState[2] = odomState[2] % (2*np.pi)
 
        print('[odom fix]', odomState, self.odom_drift)
        self.odom_state = odomState.copy()
        self.publish_state(odomState) 
        

    def publish_state(self, state):
                # we need to create a odom msg by converting pose to camera frame 
        poseMsg = Odometry()
        poseMsg.header.frame_id = "map"
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.pose.pose.position.x = state[0]
        poseMsg.pose.pose.position.y = state[1]  

        q = quaternion_from_euler(0, 0, state[2])
        poseMsg.pose.pose.orientation.x = q[0]
        poseMsg.pose.pose.orientation.y = q[1]
        poseMsg.pose.pose.orientation.z = q[2]
        poseMsg.pose.pose.orientation.w = q[3]

        self._odom_pub.publish(poseMsg)

        # publish tf so that we can see it in RVIZ 
        frameName = "/%s/odom/fix" % self.name
        self.odom_broadcast_transformation(frameName, state, q)
        

    def odom_broadcast_transformation(self, frameName, odomTagState, q):

        br = tf.TransformBroadcaster()
        br.sendTransform((odomTagState[0], odomTagState[1], 0),
                        q,
                        rospy.Time.now(),
                        frameName,
                        "map")


    def callback_ekf(self, msg:Odometry):

        # this function will operate every 1s since ekf operates at 100 hz
        if self._wait_time > 0:
            self._wait_time -= 1 
            return

        self.ekfState = odom_to_state(msg)
        if not self.initialized:
            self.initial_pos = self.ekfState.copy()
            print('[ekf initial position ]', self.initial_pos)
        
        if self.odom_initialized:
            self.odom_drift =  self.ekfState - self.odom_state
            self.odom_drift[2] = 0

        self.initialized = True
        self._wait_time = 5

class AprilTagConverter:
    def __init__(self, tags) -> None:
        self._state_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback_apriltag)
        numericTag = lambda x: int(re.findall(r'\d+', x)[0])       
        self._pose_pubs = {numericTag(tagID): rospy.Publisher('/%s/apriltag' % tagID, PoseWithCovarianceStamped, queue_size=10) for tagID in tags}
        

    def callback_apriltag(self, msg:AprilTagDetectionArray):
        for pose in msg.detections:
            self.filtered_msg(pose.id[0], pose.pose)
        
    
    def filtered_msg(self, group_id:int, poseMsg:Odometry):
        # rospy.loginfo(f"pose = {poseMsg} \n id = {group_id}") 
        # make sure the frame id is set to camera, otherwise ekf filter cannot decode it  
        poseMsg.header.frame_id = "camera"
        poseMsg.header.stamp = rospy.Time.now()
        # apriltag initial heading has 90 deg offset in z axis  
        state = odom_to_state(poseMsg)
        # by default apriltag heading is pointing in y direction 
        # we need to rotate 90 deg counter clockwise to fix this orientation problem  
        q = quaternion_from_euler(0, 0, state[-1] - np.pi/2 )
        poseMsg.pose.pose.orientation.x = q[0]
        poseMsg.pose.pose.orientation.y = q[1]
        poseMsg.pose.pose.orientation.z = q[2]
        poseMsg.pose.pose.orientation.w = q[3]
        self._pose_pubs[group_id].publish(poseMsg)

if __name__ =="__main__":
    rospy.init_node("apriltag_converter", anonymous=True)
    rospy.loginfo("[+] apriltag converted node started !!")
    tags = ['roomba20', 'roomba21']
    tag = AprilTagConverter(tags)
    odoms = [OdomFix(tag) for tag in tags]
    rospy.spin()
    # spinner = rospy.MultiThreadedSpinner(4)
    # spinner.spin()