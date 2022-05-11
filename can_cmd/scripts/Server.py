#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('can_cmd')
import actionlib

from geopy import distance
from attrdict import AttrDict

from can_cmd.PIDController import PIDLongitudinalController

from novatel_oem7_msgs.msg import BESTPOS, CORRIMU, BESTVEL
from can_cmd.msg import MoveVehicleAction, MoveVehicleResult
from can_cmd_msgs.msg import Control, Gear


conf = AttrDict({
    'subscribers':[
        {'topic':'/novatel/oem7/bestpos', 'type': BESTPOS, 'name': 'position'},
        {'topic':'/novatel/oem7/corrimu', 'type': CORRIMU, 'name': 'imu'},
        {'topic':'/novatel/oem7/bestvel', 'type': BESTVEL, 'name': 'velocity'}
        # {'topic':'/can_feedback/vehicle_speed', 'type': VEHICLESPEED, 'name': 'cur_speed'}
    ],
    'publishers':[
        {'topic':'/can_cmd/control', 'type': Control, 'name': 'control'}
    ]
})

# change parameters here
param = AttrDict({'max_speed': 8, 'max_accel': 1, 'destination': (25.230071, 126.842450),
                  'longitudinal':{'p':1.0, 'i': 0.5, 'd':0.2},
                  'dt': 0.02
                  })

global lat, lon, acc_x, cur_vel

NEUTRAL = 0
PARKING = 7
DRIVE = 5

class Controller:
    def __init__(self, conf=None, param=None):

        self.callbacks = {'/novatel/oem7/bestpos': self.pos_callback,
                          '/novatel/oem7/corrimu': self.imu_callback,
                          '/novatel/oem7/bestvel': self.spd_callback}

        self.subscribers = [rospy.Subscriber(e.topic, e.type, self.callbacks[e.topic]) for e in conf.subscribers]

        self.publishers = {e.name: rospy.Publisher(e.topic, e.type, queue_size=1) for e in conf.publishers}

        self.imu_rate = 100
        self.pub_rate = 50
        self.param = param

        self.lon_controller = PIDLongitudinalController(self.param)

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
        self.publishers['/can_cmd/control'](msg)

    # get goal from ActionClient then publish Control message to Vehicle.py['/can_cmd/vehicle']
    def run_step(self, goal):
        ct = Control()
        ct.override = True
        ct.gear = goal.gear
        cmd_vel = goal.speed
        # calculate accel from PID controller
        ct.accel_x = self.lon_controller.run_step(acc_x, cur_vel, cmd_vel)
        # for latitudinal control
        # ct.theta = self.lat_controller.run_step(*args) 
        self.publish_controls(ct)


    # send stop signal to vehicle
    def emergency_stop(self):
        pass

class VehicleControlServer:
    def __init__(self, name):
        rospy.init_node('/can_cmd/controller')
        self.name = name
        self.server = actionlib.SimpleActionServer(self.name, 
                                                   MoveVehicleAction, 
                                                   execute_cb = self.execute, 
                                                   auto_start=False)
        self.controller = Controller(conf)
        
    # get goal from ActionClient(Client)
    def execute(self, goal):
        result = MoveVehicleResult()
        rate = rospy.Rate(50) # 50Hz -> 0.02s CAN rate
        
        # received goal to stop the vehicle
        if goal.gear == PARKING:
            self.controller.run_step(goal)
        else:
            # run step while vehicle has reached 
            while abs(goal.lat - lat) < 0.000005 and abs(goal.lon - lon) < 0.000005:
                # send Control message[/can_cmd/control] to Vehicle[/can_cmd/vehicle]
                self.controller.run_step(goal)
                rate.sleep()
            rospy.loginfo("arrived at %f %f", goal.lat, goal.lon)
        result.arrived = True
        # send result to ActionClient
        self.server.set_succeeded(result)


if __name__ == '__main__':
    s = VehicleControlServer('/can_cmd/vehicle_control')
    rospy.spin()