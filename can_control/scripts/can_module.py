import cantools
import can
import threading
from pygame import time
from time import sleep


class GearStateError(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


PARKING = 0
DRIVE = 5
NEUTRAL = 6
REVERSE = 7

GEAR_STATE = [PARKING, REVERSE, NEUTRAL, DRIVE]

CONTROL_CMD = 1
DRIVING_CMD = 2

VEHICLE_INFO_1 = 3
VEHICLE_INFO_2 = 4

CYCLE_FPS = 50

dt = 0.02


class CAN:
    def __init__(self, destination=None):
        # CAN
        """ CAN Configuration Needed! """
        self.db = cantools.database.load_file("/home/nvidia/car_control/Santafe_Final.dbc")
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        self.vehicle_info_1_msg = self.db.get_message_by_name('Vehicle_info_1')
        self.vehicle_info_2_msg = self.db.get_message_by_name('Vehicle_Info_2')
        self.info_1_dict = {"APS_Feedback": 0, "Brake_ACT_Feedback": 0, "Gear_shift_Feedback": 0,
                            "Steering_angle_Feedback": 0, "Switch_state": 0}
        self.info_2_dict = {"Override_feedback": 0, "Vehicle_Speed": 0}
        self.control_cmd_msg = self.db.get_message_by_name('Control_CMD')
        self.driving_cmd_msg = self.db.get_message_by_name('Driving_CMD')
        self.control_cmd_dict = {'Override': 0, 'Alive_Count': 0, 'Angular_Speed_CMD': 30}
        self.driving_cmd_dict = {'Accel_CMD': 650, 'Brake_CMD': 0, 'Steering_CMD': 0, 'Gear_shift_CMD': PARKING}

        self.destination = destination

        self.max_speed = 7  # m/s 
        self.clock = time.Clock()

        self.cur_dist = 0

    def get_feedback(self):
        self.get_vehicle_info_1()
        self.get_vehicle_info_2()

    def send_control(self):
        self.send_driving_cmd()
        # self.send_control_cmd()

    def change_gear(self, gear):
        gear_feed = ""
        if gear == NEUTRAL:
            gear_feed = "Neutral"
        elif gear == DRIVE:
            gear_feed = "Driving"
        elif gear == PARKING:
            gear_feed = "Parking"
        elif gear == REVERSE:
            gear_feed = "Reverse"
        print("changing gear ...")
        delay = 0
        while self.info_1_dict["Gear_shift_Feedback"] != gear_feed:

            self.get_feedback()

            if self.info_2_dict["Vehicle_Speed"] <= 1 and self.info_1_dict["Brake_ACT_Feedback"] >= 900:
                if delay >= 400:
                    self.driving_cmd_dict["Gear_shift_CMD"] = gear
                    self.send_control()
                    delay = 0
                delay += 1
            else:
                self.driving_cmd_dict["Brake_CMD"] = min(self.driving_cmd_dict["Brake_CMD"] + 200, 13000)
                self.send_control()
                sleep(0.04)
                delay = 0

    def get_vehicle_info_1(self):
        try:
            msg = self.bus.recv()  # wait 0.5 sec to get msg then raise error
            if msg.arbitration_id == self.vehicle_info_1_msg.frame_id:
                data = self.db.decode_message(msg.arbitration_id, msg.data)
                self.info_1_dict = data
        except:
            return 0

    def get_vehicle_info_2(self):
        try:
            msg = self.bus.recv()  # wait 0.5 sec to get msg then raise error
            if msg.arbitration_id == self.vehicle_info_2_msg.frame_id:
                data = self.db.decode_message(msg.arbitration_id, msg.data)
                self.info_2_dict = data
        except:
            return 0

    def send_control_cmd(self):
        self.control_cmd_dict['Alive_Count'] = (self.control_cmd_dict['Alive_Count'] + 1) % 256
        data = self.control_cmd_msg.encode(self.control_cmd_dict)
        message = can.Message(arbitration_id=self.control_cmd_msg.frame_id, data=data, is_extended_id=False)
        self.bus.send(message)

    def send_driving_cmd(self):
        data = self.driving_cmd_msg.encode(self.driving_cmd_dict)
        message = can.Message(arbitration_id=self.driving_cmd_msg.frame_id, data=data, is_extended_id=False)
        self.bus.send(message)
