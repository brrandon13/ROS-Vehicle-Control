#!/usr/bin/env python3
from can_module import CAN
from pygame import time

NEUTRAL = 6
PARKING = 0
DRIVE = 5
REVERSE = 7
FPS = 50

def main():
    can = CAN()
    #can.send_control()
    while True:
        can.clock.tick_busy_loop(FPS)
        can.send_control_cmd() #alive count 
        can.get_feedback()
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("End of Alive Count ")
