#!/usr/bin/env python3
import rospy
import actionlib
from collections import deque
from can_cmd.msg import MoveVehicleAction, MoveVehicleGoal

PARKING = 7

class PathPlanner:
    def __init__(self):
        self.path = deque()
        rospy.init_node('action_client')
        # connect to /can_cmd/vehicle_control ActionServer
        self.client = actionlib.SimpleActionClient('/can_cmd/vehicle_control', MoveVehicleAction)
        
        self.init_path()

    # get path from csv file -> transform to MoveVehicleGoal objects
    def init_path(self):
        # self.path.append(MoveVehicleGoal)
        # MoveVehicleGoal: lat, lon, speed, gear
        pass
    
    # send one goal at a time to ActionServer
    def run(self):
        while self.path:
            self.client.wait_for_server()
            rospy.loginfo('sending goal ...')
            self.client.send_goal(self.path.popleft())
            # hold while waiting for result
            self.client.wait_for_result()
        # if path has ended
        else:
            parking = MoveVehicleGoal()
            parking.gear = PARKING

            parking.lat = 0 # default value
            parking.lon = 0 # default value

            parking.speed = 0
            self.client.send_goal(parking)
            
if __name__ == "__main__":
    try:
        pp = PathPlanner()
        pp.run()

    except rospy.ROSInterruptException as e:
        print('something wrong with action client:', e)

