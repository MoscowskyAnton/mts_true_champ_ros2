# !/usr/bin/env python3

import argparse
from time import sleep
from robot_driver_lib import about_control, RealRobotController, SimRobotController
import numpy as np

# import http.client
# import json
# import pdbp

# delay_time = 1.5 

def print_map(x,y):
    rmap[y][x] = 1
    m = rmap.copy()
    m[y][x] = 9
    print(m)

def calc_map_pos(d,a):
    x=0
    y=0
    if abs(a) <= 45:  # robot dir front
        if d == 'f':
            y=-1
        elif d == 'r': 
            x=+1
        elif d == 'l': 
            x=-1
        else: 
            y=1
    elif (a > 45) and (a < 135):  # robot dir right
        if d == 'f':
            x=1
        elif d == 'r': 
            y=+1
        elif d == 'l': 
            y=-1
        else: 
            x=-1
    elif (a < -45) and (a > -135):  # robot dir left
        if d == 'f':
            x=-1
        elif d == 'r': 
            y=-1
        elif d == 'l': 
            y=1
        else: 
            x=1
    else:  # back
        if d == 'f':
            y=1
        elif d == 'r': 
            x=-1
        elif d == 'l': 
            x=1
        else: 
            y=-1

    return (x,y)


rmap = np.full((16,16), 0)
robot_x = 0
robot_y = 15
rmap[robot_y][robot_x] = 1
center_door = (6,7)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', dest='sim', action='store_true', help='use simulator')
    # parser.add_argument('-m', dest='pmap', action='store_true', help='show map')
    # parser.add_argument('-r', nargs='?', default='854CAF96103A6853', help='robot id. Default = 192.168.68.167')
    parser.add_argument('-r', nargs='?', default='id1', help='robot id. Default = id1')
    parser.add_argument('-i', nargs='?', default='192.168.68.167', help='robot IP. Default = 192.168.68.167')
    parser.add_argument('-a', nargs='?', default='90', help='angle in degrees to turn. Default = 90')
    parser.add_argument('-d', nargs='?', default='180', help='dist in mm to go. Default = 180')
    parser.add_argument('-s', nargs='?', default='1.5', help='delay time in seconds. Default = 1.5')
    args = parser.parse_args()

    sim = args.sim
    # pmap = args.pmap
    pmap = False 
    robot_id = args.r
    ip = args.i
    angle = int(args.a)
    dist = int(args.d)
    delay_time = float(args.s)


    if sim:
        print('WORK ON REAL ROBOT')
        rc = SimRobotController('127.0.0.1:8801', '12345', pwm_time=500)
    else:
        print('WORK IN SIM ROBOT')
        rc = RealRobotController(ip, robot_id)

    rc.about()
    finished = False
    last_cmd = None

    while not finished:
        print(f'upd sens')
        res = rc.request_sensor_data()
        obst_around = rc.look_around()
        print()
        rc.print_sensor_data()

        # # # near door
        # if (robot_y==center_door[1]) and (robot_x==center_door[0]):
        #     while abs(rc.imu['y'] - 90) > 30:
        #         rc.send_command_mv('r', angle)
        #         sleep(delay_time)
        #         rc.request_sensor_data()
        #     rc.send_command_mv('f', dist)

        # else:
        if (obst_around['sr']): # nothing on the right
            print(f'GO RIGHT')
            dx,dy = calc_map_pos('r',rc.imu['y'])
            rc.send_command_mv('r', angle)
            sleep(delay_time)
        else: # wall on the right
            if obst_around['f']:  # nothing is in front 
                print(f'GO FWD')
                dx,dy = calc_map_pos('f',rc.imu['y'])
                pass
            else:  # wall in front 
                if (not obst_around['sl']): # wall on the left
                    print(f'GO BACK')
                    dx,dy = calc_map_pos('b',rc.imu['y'])
                    rc.send_command_mv('l', dist)
                else: 
                    print(f'GO LEFT')
                    dx,dy = calc_map_pos('l',rc.imu['y'])
                    rc.send_command_mv('l', angle)
                sleep(delay_time)

        rc.send_command_mv('f', dist)
        if pmap: 
            robot_x+=dx
            robot_y+=dy
            print_map(robot_x,robot_y)
        sleep(delay_time)

