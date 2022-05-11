#!/usr/bin/env python
import rospy
from geopy import distance
from attrdict import AttrDict
from math import sqrt

from can_cmd.PIDController import PIDLongitudinalController
from can_cmd.can_module import CAN
from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL


global lat, lon, acc_x, cur_speed

NEUTRAL = 0
PARKING = 7
DRIVE = 5

conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'},
        {'topic':'/novatel/oem7/bestvel', 'type': BESTVEL, 'name': 'velocity'}]
        # {'topic':'/can_feedback/vehicle_speed', 'type': VEHICLESPEED, 'name': 'cur_speed'}
})

param = AttrDict({'max_speed': 8, 'max_accel': 1, 'destination': (25.230071, 126.842450),
                  'longitudinal':{'p':1.0, 'i': 0.5, 'd':0.2},
                  'dt': 0.02
                  })

class Test:
    def __init__(self, conf=None, param=None):
        rospy.init_node('/can_cmd/test')

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.imu_rate = 100
        self.pub_rate = 50

        self.param = param

        self.max_accel = conf.parameters.max_accel
        self.max_speed = conf.parameters.max_speed
        self.destination = conf.parameters.destination

        self.lon_controller = PIDLongitudinalController(self.param)
        self.can = CAN()
        self.can.change_gear(NEUTRAL)
        self.can.change_gear(DRIVE)

    def target_speed(self):
        dist = distance.distance(self.destination, (lat, lon)) * 1000
        if dist < self.max_speed **2/(2*self.max_accel):
            ret = sqrt(2*self.max_accel*dist)
        else:
            ret = self.max_speed
        return ret

    def run_step(self):
        cmd_vel = self.target_speed()
        cmd_acc = self.lon_controller(cur_speed, acc_x, cmd_vel)

        if cmd_acc > 0:
            self.can.driving_cmd_dict["Brake_CMD"] = 0
            self.can.driving_cmd_dict["Accel_CMD"] = cmd_acc * 80 + 650
        elif cmd_acc < -0.2:
            self.can.driving_cmd_dict["Accel_CMD"] = 0
            self.can.driving_cmd_dict["Brake_CMD"] = -cmd_acc * 200
        else:
            self.can.driving_cmd_dict["Accel_CMD"] = 0
            self.can.driving_cmd_dict["Brake_CMD"] = 0
        
        self.can.send_control()
        self.can.get_feedback()

    
    def pos_callback(self, data):
        global lat, lon
        lat = data.lat
        lon = data.lon

    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc * self.imu_rate

    def spd_callback(self, data):
        global cur_speed
        cur_speed = data.hor_speed    

if __name__ == '__main__':
    test = Test(conf, param)
    rate = rospy.Rate(50)
    while rospy.is_shutdown():
        test.run_step()
        rate.sleep()


    