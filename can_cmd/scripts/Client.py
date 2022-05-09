#!/usr/bin/env python3
import rospy
import actionlib
from collections import deque
from can_cmd.msg import MoveVehicleAction, MoveVehicleGoal

class PathPlanner:
    def __init__(self):
        self.path = deque()
        rospy.init_node('action_client')
        self.client = actionlib.SimpleActionClient('vehicle_control', MoveVehicleAction)
        
        self.init_path()

    def init_path(self):
        # self.path.append(MoveVehicleGoal)
        # MoveVehicleGoal: lat, lon, speed
        pass

    def run(self):
        while self.path:
            self.client.wait_for_server()
            rospy.loginfo('sending goal ...')
            self.client.send_goal(self.path.popleft())
            self.client.wait_for_result()

if __name__ == "__main__":
    try:
        pp = PathPlanner()
        pp.run()

    except rospy.ROSInterruptException as e:
        print('something wrong with action client:', e)

