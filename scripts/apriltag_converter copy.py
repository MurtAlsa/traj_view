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
import tf2_ros
class AprilTagConverter:
    def __init__(self) -> None:

        self._state_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback_apriltag)
        
        # we are going to fix odometry from roomba
        # subscribe each roomba odom and publish corresponding fix odom 
        tags = ['roomba20', 'roomba21']
        numericTag = lambda x: int(re.findall(r'\d+', x)[0])
        self._odom_subs = [rospy.Subscriber('/%s/odom/'%ns, Odometry, partial(self.callback_odom, numericTag(ns))) for ns in tags]
        self._pub_odoms = {numericTag(ns):rospy.Publisher('/%s/odom/fix'%ns, Odometry, queue_size=10) for ns in tags}
        
        self._pose_pubs = {numericTag(tagID): rospy.Publisher('/%s/apriltag' % tagID, PoseWithCovarianceStamped, queue_size=10) for tagID in tags}
        
        # fixing odom requires solving initialization problem 
        # odom is initialized (0, 0) which might be problametic if robots state is in (2, 0)
        # to fix it we need a reference initialzation pose from apriltag   
        self.__initialzed = {numericTag(ns):False for ns in tags} 

        # init_param20 = rospy.get_param('%s/roomba20/init_pos'%rospy.get_name(), [2, 0, 0])
        # init_param21 = rospy.get_param('%s/roomba21/init_pos'%rospy.get_name(), [0, 0, 0])
        init_param20 = rospy.get_param('%s/roomba20/init_pos'%rospy.get_name(), [1, 0, 0])
        init_param21 = rospy.get_param('%s/roomba21/init_pos'%rospy.get_name(), [0, 1.6, 0])
        initial_pos = [init_param20, init_param21]
        self.__init_pose = {numericTag(ns):np.array(initial_pos[i])for i, ns in enumerate(tags)} 

        # we don't want to collect initial positions when tfs are not settled 
        # wait some time before fixing initial position for each robot 
        self.__start_time = time()
        self.__settling_time = 5 #s 
        self.listener = tf.TransformListener()
    
    def tf_init(self, frame_name, data):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/%s'%frame_name, rospy.Time(0))
            data[0] = 2.0 
            data[1] = 0.0
            eulers = euler_from_quaternion(rot)
            data[2] = eulers[2]
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def filtered_msg(self, group_id:int, poseMsg:Odometry):
        rospy.loginfo(f"pose = {poseMsg} \n id = {group_id}") 
        # due to calibration error real world y is 1.315 times bigger than detected y coordinate 
        # make sure the frame id is set to camera, otherwise ekf filter cannot decode it  
        poseMsg.header.frame_id = "camera"
        # poseMsg.pose.pose.position.y *= 1.315        
        state = self.odom_to_state(poseMsg)

        # by default apriltag heading is pointing in y direction 
        # we need to rotate 90 deg counter clockwise to fix this orientation problem  
        q = quaternion_from_euler(0, 0, state[-1] - np.pi/2 )
        poseMsg.pose.pose.orientation.x = q[0]
        poseMsg.pose.pose.orientation.y = q[1]
        poseMsg.pose.pose.orientation.z = q[2]
        poseMsg.pose.pose.orientation.w = q[3]
        self._pose_pubs[group_id].publish(poseMsg)
         

    def callback_apriltag(self, msg:AprilTagDetectionArray):
        for pose in msg.detections:
            self.filtered_msg(pose.id[0], pose.pose)

    @staticmethod
    def odom_to_state(poseMsg:Odometry):
        x = poseMsg.pose.pose.position.x
        y = poseMsg.pose.pose.position.y
        q = [poseMsg.pose.pose.orientation.x, poseMsg.pose.pose.orientation.y, poseMsg.pose.pose.orientation.z, poseMsg.pose.pose.orientation.w]
        state = euler_from_quaternion(q)
        return np.array([x, y, state[-1]])

    def callback_odom(self, group_id:int, msg:Odometry):
        # this callback depends on initial position of robot
        # don't publish any message until apriltags are settled
 
        odomState = self.odom_to_state(msg)
        initTag = self.__init_pose[group_id] 
        initPos = initTag[:2]
        # odom x, y coordinate needs to swap to make it to camera frame 
        tagState = odomState.copy()
        tagPos = tagState[:2]
        tagPos[0], tagPos[1] = tagPos[1], tagPos[0]
        
        
        tagPosFix =  initPos - tagPos
        tagAngleFix = tagState[-1] 

        tagState[0] = tagPosFix[0]
        tagState[1] = -tagPosFix[1]
        tagState[2] = tagAngleFix 
        

        # we need to create a odom msg by converting pose to camera frame 
        poseMsg = Odometry()
        poseMsg.header.frame_id = "map"
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.pose.pose.position.x = tagState[0]
        poseMsg.pose.pose.position.y = tagState[1]  

        q = quaternion_from_euler(0, 0, tagState[2])
        poseMsg.pose.pose.orientation.x = q[0]
        poseMsg.pose.pose.orientation.y = q[1]
        poseMsg.pose.pose.orientation.z = q[2]
        poseMsg.pose.pose.orientation.w = q[3]

        self._pub_odoms[group_id].publish(poseMsg)

        # publish tf so that we can see it in RVIZ 
        frameName = "/roomba%s/odom/fix" % group_id
        self.odom_broadcast_transformation(frameName, tagState, q)
        

    def odom_broadcast_transformation(self, frameName, odomTagState, q):

        br = tf.TransformBroadcaster()
        br.sendTransform((odomTagState[0], odomTagState[1], 0),
                        q,
                        rospy.Time.now(),
                        frameName,
                        "map")

            


if __name__ =="__main__":
    rospy.init_node("apriltag_converter", anonymous=True)
    rospy.loginfo("[+] apriltag converted node started !!")
    tag = AprilTagConverter()
    rospy.spin()

