#!/usr/bin/python3
# https://github.com/MengGuo/RVO_Py_MAS/blob/master/example.py
from RVO import RVO_update, reach, compute_V_des, reach, distance
from geometry_msgs.msg import Point, PoseStamped
import rospy
from time import sleep
from threading import Thread
from nav_msgs.msg import Odometry
from traj_view.srv import measurement
from sklearn.gaussian_process import GaussianProcessRegressor

from warnings import catch_warnings
from warnings import simplefilter
import numpy as np
import scipy.stats

np.random.seed(2135)

sensing_noise = 0.1
# sampling radius
sensing_radius = 1.0  # m
num_sample_eval = 50

# simulation parameters
target_area = [0.3, 2-0.3, -2+0.3, 0.3]



def noisy_measurement(X, var = sensing_noise):
    '''
    :param X: 2D position coordinate
    :return: z: scalaer measurement
    '''
    noise = np.random.normal(loc=0, scale=var)
    msg = Odometry()
    msg.pose.pose.position.x = X[0]
    msg.pose.pose.position.y = X[1]
    z = 0.0
    try:
        # create a handle to the add_two_ints service
        sensor_reading = rospy.ServiceProxy('sensor_reading', measurement)
        z = sensor_reading(msg)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return float(z.mes) + noise

def random(size, bound):
    '''
    :param size: number of samples
    :param bound: [Xmin, Xmax, Ymin, Ymax]
    :return: uniform random samples within the bound
    '''
    x_rand = np.random.uniform(bound[0], bound[1], size=size)
    y_rand = np.random.uniform(bound[2], bound[3], size=size)
    X = np.asarray([(x, y) for x, y in zip(x_rand, y_rand)])
    return X

# surrogate or approximation for the measurement function
def surrogate(model, X):
    # catch any warning generated when making a prediction
    with catch_warnings():
        # ignore generated warnings
        simplefilter("ignore")
        return model.predict(X, return_std=True)

# probability of improvement acquisition function
def acquisition(X, Xsamples, model):
    # calculate the best surrogate score found so far
    yhat, _ = surrogate(model, X)
    best = max(yhat)
    # calculate mean and stdev via surrogate function
    mu, std = surrogate(model, Xsamples)
    # calculate the probability of improvement
    probs = scipy.stats.norm.cdf((mu - best) / (std+1E-9))
    return probs

def opt_acquisition(X, model):

    #current state of the robot
    pos = X[-1]
    #sampling bound
    bound = [pos[0]-sensing_radius, pos[0]+sensing_radius, pos[1]-sensing_radius, pos[1]+sensing_radius]
    # random search, generate random samples
    Xsamples = random(num_sample_eval, bound)
    # point in target area
    in_target = lambda x: target_area[0] < x[0] < target_area[1] and target_area[2] < x[1] < target_area[3]
    # prune samples outside of sampling sensing_radius
    Xsamples = np.asarray([x for x in Xsamples if np.linalg.norm(x-pos)<sensing_radius and in_target(x)])
    # calculate the acquisition function for each sample
    scores = acquisition(X, Xsamples, model)
    # locate the index of the largest scores
    ix = np.argmax(scores)
    return Xsamples[ix]

class MultiAgentSystem(Thread):
    dt = 0.3
    # ------------------------------
    # define workspace model
    ws_model = dict()

    # circular obstacles, format [x,y,rad]
    # no obstacles
    ws_model['circular_obstacles'] = [ ]

    # with obstacles
    # ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
    # rectangular boundary, format [x,y,width/2,heigth/2]
    ws_model['boundary'] = []

    def __init__(self):
        super().__init__()
        self.ws_model['robot_radius'] = rospy.get_param("/%s/radius" % rospy.get_name(), 0.345)
        self.__paths = [[rospy.get_param("/%s/%s" % (rospy.get_name(), path))[0]] for path in ['robot20', 'robot21']]
        self.__cntrl_pubs = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in rospy.get_param("/%s/goal_topics" % rospy.get_name())]
        self.field_predictor = GaussianProcessRegressor()
        self.__readings = [[noisy_measurement(x[0])] for x in self.__paths]
        # print(self.__readings)

    @staticmethod
    def publish_setpoint(setpoint, pub):
        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        pub.publish(pose)

    def next_goal_location(self, X):
        for i, goal in enumerate(X):
            X[i] = opt_acquisition(self.__paths[i], self.field_predictor)
            self.__paths[i].append(X[i].tolist())
            self.__readings[i].append(noisy_measurement(X[i]))

        # update filed predictor
        path = np.array(self.__paths)
        path = path.reshape((path.shape[0] * path.shape[1], path.shape[2]))
        Z = np.array(self.__readings)
        Z = Z.reshape((Z.shape[0] * Z.shape[1]))
        # print(Z.shape, path.shape)
        rospy.loginfo(f"sample size = {path.shape} ")
        self.field_predictor.fit(path, Z)

        return X


    def run(self):
        X = [path[0] for path in self.__paths]
        # velocity of [vx,vy]
        V = [[0, 0] for _ in range(len(X))]
        # maximal velocity norm
        V_max = [1.0 for _ in range(len(X))]
        # goal of [x,y]
        goal = X.copy()
        # delay function
        rate = rospy.Rate(self.dt)

        while not rospy.is_shutdown():
            goal = self.next_goal_location(goal)
            # compute desired vel to goal
            V_des = compute_V_des(X, goal, V_max)
            # compute the optimal vel to avoid collision
            V = RVO_update(X, V_des, V, self.ws_model)

            # update position
            for i in range(len(X)):
                X[i][0] += V[i][0] * self.dt
                X[i][1] += V[i][1] * self.dt
                # check i th robot reach the goal location or not
                self.publish_setpoint(X[i], self.__cntrl_pubs[i])
            rate.sleep()
        print("[+] terminated")
        sleep(5)


if __name__ == '__main__':
    rospy.init_node('roomba_traj_viewer', anonymous=True)
    rospy.get_namespace()

    mas = MultiAgentSystem()
    print('waiting for sensor_reading')
    rospy.wait_for_service('sensor_reading')
    print('node started')
    mas.start()
    mas.join()
