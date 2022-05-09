#!/usr/bin/env python3
import rospy
from can_cmd_msgs.msg import Control, Gear
from can_module import CAN

DRIVE =  5
PARKING = 0

class Vehicle:
    def __init__(self):
        self.can = CAN()
        rospy.init_node('vehicle', anonymous=True)
        rospy.Subscriber('control', Control, self.callback)

        self.can.change_gear(DRIVE)
    
    def callback(self, data):
        

        if data.accel_x > 0:
            self.can.driving_cmd_dict["Accel_CMD"] = data.accel_x * 80 + 650
        else:
            self.can.driving_cmd_dict["Brake_CMD"] = -data.accel_x * 1000

if __name__ == '__main__':
    v = Vehicle()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        v.can.send_control()
        v.can.get_feedback()
        rate.sleep()

