#!/usr/bin/env python3

from pickle import FALSE
import rospy

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from collections import deque
from can_module import CAN


from geopy import distance
from attrdict import AttrDict

from PIDController import PIDLongitudinalController
from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL, HEADING2

conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'},
        {'topic':'/novatel/oem7/bestvel', 'type': BESTVEL, 'name': 'velocity'},
        {'topic':'/novatel/oem7/heading2', 'type': HEADING2, 'name': 'yaw'}
    ]})

# change parameters here
param = AttrDict({'max_speed': 5, 'max_accel': 0.5, 'destination': (35.22555668, 126.83843677),
                  'longitudinal':{'p':15, 'i': 0, 'd':2},
                  'dt': 0.02,
                  'lateral':{'K':0.5}
                  })

lat = 0
lon = 0 
acc_x = 0
cur_vel = 0 
yaw = 0


NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7

class Controller:
    def __init__(self, conf=None, param=None):
        rospy.init_node('controller', anonymous=True)

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback,
                          '/novatel/oem7/heading2': self.yaw_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.imu_rate = 100
        self.pub_rate = 10
        self.param = param

        self.max_speed = param.max_speed
        self.max_accel = param.max_accel

        self.lon_controller = PIDLongitudinalController(param)
        #self.lat_controller = LateralController(param)
        self.can = CAN()

        """ 시작점, 끝점 """
        self.start_pos = (35.22510137, 126.83975056)
        self.target_pos = (35.22545213, 126.83870505)

        self.max_travel_distance = distance.distance(self.start_pos, self.target_pos)

        self.path = []
        self.index = 0
        #self.init_path()
        #self.initial_gear()

        self.cur_vel_log = []
        self.cmd_vel_log = []
        self.yaw_log = []

    """ load path or get update from other module """
    def init_path(self):
        self.path = [(35.22994788,126.842817,4.242640687,5),
        (35.22999963,126.842673,7.071067812,5),
        (35.23033928,126.841728,8,5),
        (35.23053336,126.841188,4.898979486,5),
        (35.230556,126.841125,3,5)]

        # target = (, 126.842448822, 0, 5)
        # self.path.append(target)
        # target = (35.2303007467, 126.841890354, 0, 5)
        # self.path.append(target)
        # self.start_pos = (35.22510915, 126.38975105)

    def initial_gear(self):
        while self.can.info_1_dict["Gear_shift_Feedback"] == 0:
            self.can.get_feedback()
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Parking":
            self.can.change_gear(REVERSE)
            print('1') 
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Reverse":
            self.can.change_gear(NEUTRAL)
            print('2')        
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Neutral":
            self.can.change_gear(DRIVE)
            print('3')
        if self.can.info_1_dict["Gear_shift_Feedback"] == "Driving":
            print('Current Driving')
        
        print("Initial Gear")

    def pos_callback(self, data):
        global lat, lon
        lat = data.lat
        lon = data.lon

    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc / self.imu_rate

    def spd_callback(self, data):
        global cur_vel
        cur_vel = data.hor_speed 
    
    def yaw_callback(self, data):
        # 값 개튐
        global yaw
        yaw = data.heading

    """ when using starting position and target position. Trapezoidal velocity profile """
    def get_target_velocity(self):
        dist = float(distance.distance((lat, lon), self.start_pos).km)
        tar_dist = float(distance.distance((lat, lon), self.target_pos).km)
        if dist > self.max_travel_distance:
            return 0
        if dist * 1000 < 0.5 * self.max_speed ** 2 / self.max_accel:
            return sqrt(2 * self.max_accel * dist * 1000)
        if tar_dist * 1000 < 0.5 * self.max_speed ** 2 / self.max_accel:
            return sqrt(2 * self.max_accel * abs(tar_dist) * 1000)
        else:
            return self.max_speed

    def run_step(self):
        """ when using path """
        # tar_lat, tar_lon = self.path[self.index][0], self.path[self.index][1]
        # tar_vel = self.path[self.index][2]
        # if distance.distance((lat, lon), (tar_lat, tar_lon)) < 0.0005:
        #     self.index += 1

        tar_vel = self.get_target_velocity()

        cmd_acc = self.lon_controller.run_step(cur_vel, tar_vel, acc_x)
        cmd_steer = 0

        self.send_can_control(cmd_acc, cmd_steer)

    def send_can_control(self, acc, steer):
        """ steer not added yet """
        if acc > 0:
            self.can.driving_cmd_dict["Brake_CMD"] = 0
            self.can.driving_cmd_dict["Accel_CMD"] = 650 + int(acc * 55)
        elif acc > -0.2:
            self.can.driving_cmd_dict["Accel_CMD"] = 650
            self.can.driving_cmd_dict["Brake_CMD"] = 0 
        else:
            self.can.driving_cmd_dict["Accel_CMD"] = 650
            self.can.driving_cmd_dict["Brake_CMD"] = min(int(-acc * 1000),13000)
        self.can.send_control()

    def reached_destination(self):
        if distance.distance(self.start_pos, (lat, lon)) > self.max_travel_distance:
            return True
        if distance.distance((lat, lon), self.target_pos).km > 0.001:
            return False
        else:
            print("reached destination")
            return True

    def make_plot(self):
        plt.figure(1)
        plt.plot(list(range(len(self.cur_vel_log))), self.cur_vel_log)
        plt.plot(list(range(len(self.cmd_vel_log))), self.cmd_vel_log)
        plt.show()
        plt.figure(2)
        plt.plot(list(range(len(self.yaw_log))), self.yaw_log)
        plt.show()


if __name__ == '__main__':
    vehicle = Controller(conf, param)
    try:
        rate = rospy.Rate(50)

        while not vehicle.reached_destination() and not rospy.is_shutdown():
            vehicle.run_step()
            rate.sleep()

        else:
            print("now braking")
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Driving":
                print('Current Driving')
                vehicle.can.change_gear(NEUTRAL)
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Neutral":
                vehicle.can.change_gear(REVERSE)
            if vehicle.can.info_1_dict["Gear_shift_Feedback"] == "Reverse":
                vehicle.can.change_gear(PARKING)
        print("Done")

    except KeyboardInterrupt:
        vehicle.send_can_control(-11, 0)
        print("end control")
