#!/usr/bin/env python3
import rospy
import roslib
roslib.load_manifest('can_cmd')
import actionlib

from geopy import distance
from attrdict import AttrDict

from PIDController import PIDLongitudinalController

from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL
from can_cmd.msg import MoveVehicleAction, MoveVehicleResult
from can_cmd_msgs.msg import Control, Gear


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


global lat, lon, acc_x, cur_vel

class Controller:
    def __init__(self, conf=None):

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback}

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

        self.lat_controller = PIDLongitudinalController(self.longitudinal_args)

    def pos_callback(self, data):
        global lat, lon
        lat = data.lat
        lon = data.lon

    def imu_callback(self, data):
        global acc_x
        acc_x = data.longitudinal_acc * self.imu_rate

    def spd_callback(self, data):
        global cur_vel
        cur_vel = data.hor_speed    

    def publish_controls(self, msg):
        self.publishers['control'](msg)

    def run_step(self, speed):
        ct = Control()
        ct.override = True
        ct.gear = Gear.DRIVE

        cmd_vel = speed
        ct.accel_x = self.lat_controller.run_step(acc_x, cur_vel, cmd_vel)
        # ct.theta = self.lon_controller.run_step(*args)
        self.publish_controls(ct)

    def stop_vehicle(self):
        pass

class VehicleControlServer:
    def __init__(self, name):
        rospy.init_node('controller')
        self.name = name
        self.server = actionlib.SimpleActionServer(self.name, 
                                                   MoveVehicleAction, 
                                                   execute_cb = self.execute, 
                                                   auto_start=False)
        self.controller = Controller()
        rospy.spin()

    def execute(self, goal):
        result = MoveVehicleResult()
        rate = rospy.Rate(50) # 
        while abs(goal.lat - lat) < 0.000005 and abs(goal.lon - lon) < 0.000005:
            self.controller.run_step(goal.speed)
            rate.sleep()
        rospy.loginfo(f'arrived at {goal.lat} {goal.lon}')
        result.arrived = True
        self.server.set_succeeded(result)


if __name__ == '__main__':
    s = VehicleControlServer('vehicle_control')