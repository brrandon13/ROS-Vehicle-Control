#!/usr/bin/env python
import rospy
from can_cmd_msgs.msg import Control, Gear
from can_cmd.can_module import CAN

DRIVE =  5
NEUTRAL = 7
PARKING = 0

global gear, acc

class Vehicle:
    def __init__(self):
        self.can = CAN()
        rospy.init_node('/can_cmd/vehicle', anonymous=True)

        self.can.change_gear(NEUTRAL)
        self.can.change_gear(DRIVE)

        rospy.Subscriber('/can_cmd/control', Control, self.callback)
    
    # subscribe to Control message then send CAN messages accordingly
    def callback(self, data):
        if data.override:
            self.can.control_cmd_dict["Override_Off"] = 0
        else:
            self.can.control_cmd_dict["Override_Off"] = 1
            
        if data.gear != self.info_1_dict["Gear_Shift_Feedback"]:
            self.can.change_gear(NEUTRAL)
            self.can.change_gear(data.gear)
        else:
            if data.accel_x > 0:
                self.can.driving_cmd_dict["Brake_CMD"] = 0  
                self.can.driving_cmd_dict["Accel_CMD"] = data.accel_x * 80 + 650
            else:
                self.can.driving_cmd_dict["Accel_CMD"] = 0
                self.can.driving_cmd_dict["Brake_CMD"] = -data.accel_x * 1000
            self.can.send_control()
            self.can.get_feedback()

if __name__ == '__main__':
    v = Vehicle()
    rospy.spin()

