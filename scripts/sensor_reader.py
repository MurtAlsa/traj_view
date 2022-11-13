#!/usr/bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from traj_view.srv import measurement, measurementResponse


class Measurement:
    def __init__(self, map_topic_name):
        self._map_sub = rospy.Subscriber(map_topic_name, OccupancyGrid, self.read_map)
        self._service = rospy.Service('sensor_reading', measurement, self.handle_mes_req)

    def read_map(self, msg: OccupancyGrid):
        rospy.loginfo(msg.info)
        self.height = msg.info.height
        self.width = msg.info.width
        self.resolution = msg.info.resolution
        data = np.array(msg.data, dtype=int)
        self.__data = np.reshape(data, (self.width, self.height))
        self.__data -= np.min(np.min(self.__data, axis=1))
        self.__origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        print(self.__origin)

    def scale(self, x, x_min, y_min, x_max, y_max):
        x_std = (x - x_min) / (x_max - x_min)
        return int(x_std * (y_max - y_min))

    def __call__(self, msg: Odometry):
        # x_range [0, 2] y_range [0, -2]
        x = self.scale(msg.pose.pose.position.x, self.__origin[0], 0, self.width * self.resolution, self.width)
        y = self.scale(msg.pose.pose.position.y, self.__origin[1], 0, 0, self.height)

        x = max(min(self.width - 1, x), 0)
        y = max(min(self.height - 1, y), 0)
        return self.__data[x, y]

    def handle_mes_req(self, req):
        return measurementResponse(self(req.pose))


if __name__ == '__main__':
    rospy.init_node('SensorReader', anonymous=True)
    mes = Measurement('map')
    rospy.spin()
