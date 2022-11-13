#!/usr/bin/python3
import ros
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import tf

np.set_printoptions(precision=4)
# controller fq in Hz
rate_cntrl_update = 30

class TrajController:
    dt = 1/rate_cntrl_update

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        # init_pos = rospy.get_param('/%s/init_pos' % rospy.get_name())
        init_pos = [0, 0, 0]
        self._target_state = np.array(init_pos)
        self._curr_state = np.array(init_pos, dtype=float)
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

        ns = rospy.get_namespace()
        print("name space:", ns)
        state_topic = "%sodometry/filtered" % ns
        cmd_vel_topic = "%scmd_vel" % ns
        cmd_vel_ekf_topic = "%scmd_vel_ekf" % ns
        goal_topic = "%smove_base_simple/goal" % ns 
        viz_pub_topic = "%spath" % ns
        self.roomba_name = ns

        self.__initialized = False

        # define subscriber 
        self._goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.goal_update)
        self._state_sub = rospy.Subscriber(state_topic, Odometry, self.state_callback)

        # define publisher 
        self._path_pub = rospy.Publisher(viz_pub_topic, Marker, queue_size=10)
        self._cmd_pub_ekf = rospy.Publisher(cmd_vel_ekf_topic, TwistWithCovarianceStamped, queue_size=10)
        self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10) 
        self.__wait = 0.0
        self.__wait_thres = 2.0 # s
        self.__adaptive_thres = 0.1
        self.listener = tf.TransformListener()
        
    def complimentaryFilter(self, state):
        if not self.__initialized:
            self.__state = state
            self.__initialized = True 
        alpha = 0.8
        N = len(state)
        for i in range(N):
            self.__state[i] = self.__state[i] * alpha + (1 - alpha) * state[i]
        
        return self.__state.copy()

    def state_callback(self, msg:Odometry):

        # eulers = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # state = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulers[2]]
        # state = self.complimentaryFilter(state)
        
        # self._curr_state[0] = state[0]
        # self._curr_state[1] = state[1]
        # self._curr_state[2] = state[2]
        
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '%sbase_link'%self.roomba_name, rospy.Time(0))
            eulers = euler_from_quaternion(rot)
            state = [trans[0], trans[1], eulers[2]]
            # state = self.complimentaryFilter(state)
            # self._curr_state[0] = state[0]
            # self._curr_state[1] = state[1]
            # self._curr_state[2] = state[2]

            self._curr_state[0] = trans[0]
            self._curr_state[1] = trans[1]            
            self._curr_state[2] = eulers[-1]              
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        print("controller state update ", self._curr_state)

        if (self.__wait <= self.__wait_thres):
            self._target_state = self._curr_state.copy()


        # print(self._curr_state)
        marker = self.current_state_pub(self._curr_state)
        self._path_pub.publish(marker)

    def goal_update(self, msg: PoseStamped):
        """
        trajectory view node publish goal position that will be subscribed here
        :param msg: PoseStamp message
        :return: update internal variable target state
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        if msg.header.frame_id == "last":
            self.__adaptive_thres = 0.15
            print("last frame received")

        dx = x - self._curr_state[0]
        dy = y - self._curr_state[1]
        theta = np.arctan2(dy, dx)
        self._target_state = np.array([x, y, theta], dtype=float)
        print("target state ", self._target_state)

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal
        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis
        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w

    @staticmethod
    def current_state_pub(state):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        ns = rospy.get_namespace()
        marker.ns = '/%s/controlState' % ns
        marker.id = 202
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.type = Marker.MESH_RESOURCE
        # marker.mesh_resource = "file:///home/redwan/Blender/ROOMBA.dae"
        marker.mesh_resource = "package://traj_view/config/ROOMBA.dae"
        # set up scale of the points
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.50
        marker.color.r = 0.663
        marker.color.g = 0.663
        marker.color.b = 0.663

        marker.pose.position.x = state[0]
        marker.pose.position.y = state[1]
        marker.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, state[-1])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        return marker

    def cntrl_update(self, event):
        """
        :param event: timer event
        :return:compute control parameters
        """
        self.__wait += self.dt 
        state_diff = self._target_state - self._curr_state
        rho, v, w = self.calc_control_command(state_diff[0], state_diff[1], self._curr_state[-1],
                                              self._target_state[-1])

        
        clip = lambda val, clip_val: min(abs(val), clip_val) * np.sign(val)
        # v = clip(v, 1.5)
        # w = clip(w, 1.0)

        if rho <= self.__adaptive_thres or self.__wait <= self.__wait_thres:
            v, w = 0, 0

        msg_cov = TwistWithCovarianceStamped()
        msg = Twist()
        msg_cov.header.stamp = rospy.Time.now()
        msg_cov.header.frame_id = 'map'

        msg_cov.twist.twist.linear.x = v
        msg_cov.twist.twist.angular.z = w
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub_ekf.publish(msg_cov)
        self._cmd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("TrajController", anonymous=True)
    rospy.loginfo("Traj Controller Initiated")
    # trajCntrl = TrajController(0.3, 0.5, 0.0)
    trajCntrl = TrajController(0.9, 1.5, 0.0)
    # trajCntrl = TrajController(0.3, 0.6, 0.0)
    
    rospy.Timer(rospy.Duration(1.0 / rate_cntrl_update), trajCntrl.cntrl_update)
    rospy.spin()
