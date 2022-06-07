#!/usr/bin/env python3

from attrdict import AttrDict
from collections import deque
import numpy as np
import pandas as pd
from math import sqrt
from math import radians



NORMAL = 1
BACK = 2
EXP = 3

class PIDLongitudinalController:
    def __init__(self, args) -> None:
        self.max_speed = args.max_speed
        self.max_accel = args.max_accel
        self.N_p = args.normal_pid_gain.p
        self.N_i = args.normal_pid_gain.i
        self.N_d = args.normal_pid_gain.d
        self.B_p = args.back_pid_gain.p
        self.B_i = args.back_pid_gain.i
        self.B_d = args.back_pid_gain.d
        self.E_p = args.exp_pid_gain.p
        self.E_i = args.exp_pid_gain.i
        self.E_d = args.exp_pid_gain.d
        self._dt = args.dt
    
        self._buffer = deque(maxlen=10)

    def run_step(self, cur_vel, cmd_vel, mode):
        if mode == NORMAL:
            error = cmd_vel - cur_vel
            self._buffer.append(error)

            if len(self._buffer) >= 2:
                de = (self._buffer[-1] - self._buffer[-2])/self._dt
                ie = sum(self._buffer) * self._dt
            else:
                de = 0
                ie = 0

            cmd = (self.N_p * error) + (self.N_d * de) + (self.N_i * ie)
            #print("command accel: ", cmd_acc, "target velocity: ", cmd_vel, "cur velocity: ", cur_vel)
            return cmd # 4가 원래 스피드
        
        elif mode == BACK:
            error = cmd_vel - cur_vel
            self._buffer.append(error)

            if len(self._buffer) >= 2:
                de = (self._buffer[-1] - self._buffer[-2])/self._dt
                ie = sum(self._buffer) * self._dt
            else:
                de = 0
                ie = 0

            cmd_acc = (self.B_p * error) + (self.B_d * de) + (self.B_i * ie)
            print("command accel: ", cmd_acc, "target velocity: ", cmd_vel, "cur velocity: ", cur_vel)
            return min(cmd_acc, 8)

        elif mode == EXP:
            error = cmd_vel - cur_vel
            self._buffer.append(error)

            if len(self._buffer) >= 2:
                de = (self._buffer[-1] - self._buffer[-2])/self._dt
                ie = sum(self._buffer) * self._dt
            else:
                de = 0
                ie = 0

            cmd_acc = (self.E_p * 1/error) + (self.E_d * de) + (self.E_i * ie)
            #print("command accel: ", cmd_acc, "target velocity: ", cmd_vel, "cur velocity: ", cur_vel)
            return min(cmd_acc, 13)   # 13 급정거


class LateralController:
   
   def normalize_angle(self,angle):
        if angle > np.pi:
             angle -= 2.0 * np.pi

        elif angle < -np.pi:
                angle += 2.0 * np.pi

        return angle


   def stanley(self, cur_vel,cur_lat, cur_lon, cur_yaw, tar_lat, tar_lon ,tar_yaw):
        # find nearest point
        min_dist = 0.0002
        k = 0.5

        # compute cte at front axle
        dx = tar_lat - cur_lat
        dy = tar_lon - cur_lon
        dist = np.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
                min_dist = dist



        perp_vec = [np.cos(np.pi/180*cur_yaw + np.pi/2), np.sin(np.pi/180*cur_yaw + np.pi/2)]
        cte = np.dot([dx, dy], perp_vec)

        # control lawxa
        yaw_E = tar_yaw-np.pi/180*cur_yaw
        # print("YAWEEEE: ", yaw_E, "cur yaw: ", cur_yaw, "tar_yaw", tar_yaw)
        yaw_term = self.normalize_angle(yaw_E)
        cte_term = 180/np.pi*np.arctan2(k*cte,-cur_vel*0.0002778)

        # steering
        steer = int(yaw_term*180/np.pi + cte_term)
        print( "cur_yaw : ",cur_yaw,"tar_yaw : ",tar_yaw,)


        return np.clip(steer, -45, 45)
