#!/usr/bin/env python3
import rospy
from collections import deque
import csv
from geopy import distance
from novatel_oem7_msgs.msg import BESTPOS, CORRIMU
# from can_feedback_msgs.msg import VEHICLESPEED # cmakelist + package
from can_cmd_msgs.msg import Control
from attrdict import AttrDict
from PIDController import PIDLongitudinalController


conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'}
        # {'topic':'/can_feedback/vehicle_speed', 'type': VEHICLESPEED, 'name': 'cur_speed'}
    ],
    'publishers':[
        {'topic':'control', 'type': Control, 'name': 'control'}
    ]
})


global lat, lon, acc_x, cur_speed

class Controller:
    def __init__(self, conf=None):
        rospy.init_node('controller', anonymous=True)

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/can_feedback/vehicle_speed': self.spd_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.publishers = {e.name: rospy.Publisher(e.topic, e.type, queue_size=1) for e in conf.publishers}

        self.imu_rate = 100
        self.pub_rate = 50

        self.longitudinal_args = AttrDict({'max_speed': rospy.get_param('/can_cmd/max_speed', 8),
                                            'max_accel': rospy.get_param('/can_cmd/max_accel', 2.0), 
                                            'p'        : rospy.get_param('/can_cmd/longitudinal/P', 1.0), 
                                            'i'        : rospy.get_param('/can_cmd/longitudinal/I', 0.5), 
                                            'd'        : rospy.get_param('/can_cmd/longitudinal/D', 0.1),
                                            'dt'       : self.pub_rate})

        self.path = deque()

        self.init_path(open("path.csv", "r"))
        self.lat_controller = PIDLongitudinalController(self.longitudinal_args)

    def pos_callback(self, data):
        global lat, lon
        lat = data.lat
        lon = data.lon

    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc * self.imu_rate

    def spd_callback(self, data):
        global cur_speed
        cur_speed = data.speed    

    def init_path(self, file):
        csv_reader = csv.reader(file)

        for row in csv_reader:
            self.path.append(tuple(map(float, row)))

    def publish_controls(self):
        self.publishers['control'](self.run_step())

    def run_step(self):
        ct = Control()
        cmd_vel = self.path[0][-1]
        if distance.distance((lat, lon), self.path[:2]) < 0.001: # less than 1 m
            self.path.popleft()
        ct.accel_x = self.lat_controller.run_step()