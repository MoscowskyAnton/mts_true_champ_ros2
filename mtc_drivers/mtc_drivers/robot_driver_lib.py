# !/usr/bin/env python3

import argparse
import http.client
import json
from time import sleep

from rclpy.node import Node

import pdbp


# Optional for ROS logging

def universal_print(msg="", ros_node=None):
    """
    Uses ROS2 to print if given the ROS2 node, otherwise just regular print
    :param msg:
    :param ros_node:
    :return:
    """
    if ros_node:
        ros_node.get_logger().info(msg)
    else:
        print(msg)


def log_request(method, url, body, headers):
    universal_print(f"--- BEGIN REQUEST ---")
    universal_print(f"{method} {url} HTTP/1.1")
    for header, value in headers.items():
        universal_print(f"{header}: {value}")
    if body:
        universal_print(f"\n{body.decode('utf-8') if isinstance(body, bytes) else body}")
    universal_print(f"--- END REQUEST ---")


class DebugHTTPConnection(http.client.HTTPConnection):
    """
    USE ONLY FOR TESTING, NOT WHOLE REQUEST IS IMPLEMENTED WHEN PASSED TO SUPER
    """

    def request(self, method, url, body=None, headers=None, **kwargs):
        if headers is None:
            headers = {}
        # Print the full HTTP request
        log_request(method, url, body, headers)

        # Call the original request method
        super().request(method, url, body, headers, **kwargs)


class RealRobotController:
    def __init__(
            self,
            ip_=None,
            robot_id_=None,
            pwm_val=100,
            pwm_time=100,
            execute_cmd_=True,
            event_registrator=None,
            node: Node = None
    ):
        # Optional ROS node to enable ROS logging
        self.node: Node = node

        self.actions = {'f': 'forward', 'b': 'backward', 'l': 'left', 'r': 'right'}
        self.laser = {'f': 0, 'b': 0, 'l': 0, 'r': 0, 'sl': 0, 'sr': 0}
        self.imu = {'r': 0, 'p': 0, 'y': 0}
        self.sens_types = ['all', 'laser', 'imu']
        self.orients = {'f': 0, 'b': 180, 'l': -90, 'r': 90}
        self.ip = ip_
        self.robot_id = robot_id_
        self.pwm_val = pwm_val
        self.pwm_time = pwm_time
        self.execute_cmd = execute_cmd_
        self.delay_time = 0.05
        self.sens_lim = 50
        self.error_turn = 8
        self.robot_orient = None

        self.event_registrator = event_registrator

        universal_print(f"RealRobotController uses ip={self.ip} and robot_id={self.robot_id}")

    def about(self):
        universal_print(f'=== robot param ===')
        universal_print(f'ip: {self.ip}')
        universal_print(f'id: {self.robot_id}')
        universal_print()
        universal_print('man ctl param:')
        universal_print(f'pwm_val: {self.pwm_val}')
        universal_print(f'pwm_time: {self.pwm_time}')
        universal_print()
        universal_print('sensors param:')
        universal_print(f' l  f  r')
        universal_print(f'sl  -  sr')
        universal_print(f'    b   ')
        universal_print()
        universal_print()

    def request_sensor_data(self, sens_type='all'):
        assert (sens_type in self.sens_types)
        # conn = DebugHTTPConnection(self.ip)
        conn = http.client.HTTPConnection(self.ip)
        headers = {'Content-Type': 'application/json'}
        # params = {'id': f'{self.robot_id}', 'type':f'{sens_type}'}
        params = '{"id": "' + str(self.robot_id) + '", "type":"' + str(sens_type) + '"}'

        if self.execute_cmd:
            conn.request('POST', f'/sensor', body=params, headers=headers)
            # DEBUG ONLY
            if self.event_registrator is not None:
                self.event_registrator.add_debug_data('POST sensor ' + str(params))
            # END DEBUG ONLY
            resp = conn.getresponse()
            if resp.status != 200:
                universal_print("Failed")
                return False, (resp.status, resp.reason), {}
            else:
                resp_data = resp.read()
                universal_print(f"resp_data {resp_data}")
                parsed_resp = json.loads(resp_data)
                universal_print(parsed_resp)
                if sens_type in ['all', 'laser']:
                    self.laser['f'] = int(parsed_resp['laser']['4'])
                    self.laser['b'] = int(parsed_resp['laser']['1'])
                    self.laser['sr'] = int(parsed_resp['laser']['5'])
                    self.laser['sl'] = int(parsed_resp['laser']['2'])
                    self.laser['r'] = int(parsed_resp['laser']['3'])
                    self.laser['l'] = int(parsed_resp['laser']['6'])

                if sens_type in ['all', 'imu']:
                    self.imu['r'] = parsed_resp['imu']['roll']
                    self.imu['p'] = parsed_resp['imu']['pitch']
                    self.imu['y'] = int(parsed_resp['imu']['yaw'] - 180)  # it is in [0,360]
                    if abs(self.imu['y']) <= 45:  # front
                        self.robot_orient = 'f'
                    elif (self.imu['y'] > 45) and (self.imu['y'] < 135):  # right
                        self.robot_orient = 'r'
                    elif (self.imu['y'] < -45) and (self.imu['y'] > -135):  # left
                        self.robot_orient = 'l'
                    else:  # back
                        self.robot_orient = 'b'

                return True, (resp.status, resp.reason), parsed_resp
        else:
            universal_print(f'{params}')

    # param: if direction = f/b - dist in mm
    # param: if direction = r/l - angle in gradus
    def send_command_mv(self, direction, param):
        assert (direction in self.actions.keys())
        conn = DebugHTTPConnection(self.ip)
        headers = {'Content-Type': 'application/json'}
        # params = {'id': f'{self.robot_id}', 'direction':f'{self.actions[direction]}', 'len':param}
        params = '{"id": "' + str(self.robot_id) + '", "direction":"' + str(
            self.actions[direction]) + '", "len":' + str(param) + '}'
        if self.execute_cmd:
            conn.request('PUT', f'/move', body=params, headers=headers)
            # DEBUG ONLY
            if self.event_registrator is not None:
                self.event_registrator.add_debug_data('PUT move ' + str(params))
            # END DEBUG ONLY

            resp = conn.getresponse()
            return resp.status == 200
        else:
            universal_print(f'{params}')

    def send_command_motor(self, l, r, l_time, r_time):
        conn = DebugHTTPConnection(self.ip)
        headers = {'Content-Type': 'application/json'}
        # params = {'id': f'{self.robot_id}', 'l':l, 'r':r,'l_time':l_time, 'r_time':r_time}
        params = '{"id": "' + str(self.robot_id) + '", "l": ' + str(l) + ', "r": ' + str(r) + ',"l_time": ' + str(
            l_time) + ', "r_time": ' + str(r_time) + '}'
        if self.execute_cmd:
            conn.request('PUT', f'/motor', body=params, headers=headers)
            # DEBUG ONLY
            if self.event_registrator is not None:
                self.event_registrator.add_debug_data('PUT movor ' + str(params))
            # END DEBUG ONLY

            resp = conn.getresponse()
            return resp.status == 200
        else:
            universal_print(f'{params}')

    def look_around(self):  # what does robot see around
        return {'f': self.laser['f'] > self.sens_lim, 'b': self.laser['b'] > self.sens_lim,
                'l': self.laser['l'] > self.sens_lim,
                'r': self.laser['r'] > self.sens_lim, 'sl': self.laser['sl'] > self.sens_lim,
                'sr': self.laser['sr'] > self.sens_lim}

    def print_sensor_data(self):
        universal_print(f'laser: {self.laser}')
        universal_print(f'imu: {self.imu}')
        # universal_print(f'look_around: {self.look_around()}')
        universal_print(f'orient: {self.robot_orient}')
        universal_print()


class SimRobotController(RealRobotController):
    def __init__(self, ip_=None, robot_id_=None, pwm_val=100, pwm_time=100, execute_cmd_=True):
        super().__init__(ip_, robot_id_, pwm_val, pwm_time, execute_cmd_)
        self.sens_lim = 86

    def request_sensor_data(self, sens_type='all'):
        assert (sens_type in self.sens_types)
        conn = DebugHTTPConnection(self.ip)
        headers = {'accept': 'application/json'}
        params = ''
        if self.execute_cmd:
            conn.request('GET', f'/api/v1/robot-cells/sensor-data?token={self.robot_id}', body=params, headers=headers)
            resp = conn.getresponse()
            if resp.status != 200:
                return False, (resp.status, resp.reason), {}
            else:
                parsed_resp = json.loads(resp.read())
                self.laser['f'] = parsed_resp['front_distance']
                self.laser['b'] = parsed_resp['back_distance']
                self.laser['sr'] = parsed_resp['right_side_distance']
                self.laser['sl'] = parsed_resp['left_side_distance']
                self.laser['r'] = parsed_resp['right_45_distance']
                self.laser['l'] = parsed_resp['left_45_distance']

                # ==========================================================
                # might be a problem: unknown limits - [0,360) or [-180,180)
                # ==========================================================
                self.imu['r'] = parsed_resp['rotation_roll']
                self.imu['p'] = parsed_resp['rotation_pitch']
                self.imu['y'] = parsed_resp['rotation_yaw']
                if abs(self.imu['y']) <= 45:  # front
                    self.robot_orient = 'f'
                elif (self.imu['y'] > 45) and (self.imu['y'] < 135):  # right
                    self.robot_orient = 'r'
                elif (self.imu['y'] < -45) and (self.imu['y'] > -135):  # left
                    self.robot_orient = 'l'
                else:  # back
                    self.robot_orient = 'b'

                return True, (resp.status, resp.reason), parsed_resp
        else:
            universal_print(f'{params}')

    def send_command_mv(self, direction, param):
        assert (direction in self.actions.keys())
        conn = DebugHTTPConnection(self.ip)
        headers = {'accept': 'application/json'}
        params = ''
        if self.execute_cmd:
            conn.request('POST', f'/api/v1/robot-cells/{self.actions[direction]}?token={self.robot_id}', body=params,
                         headers=headers)
            resp = conn.getresponse()
            return resp.status == 200
        else:
            universal_print(f'{params}')

    def send_command_motor(self, l, r, l_time, r_time):
        conn = DebugHTTPConnection(self.ip)
        headers = {'accept': 'application/json'}
        params = ''
        l_time *= 0.001  # in sim API cmd in seconds
        r_time *= 0.001
        if self.execute_cmd:
            conn.request('POST', f'/api/v1/robot-motors/move?token={self.robot_id}'
                                 f'&l={l}&l_time={l_time}&r={r}&r_time={r_time}', headers=headers)
            resp = conn.getresponse()
            sleep(self.delay_time)
            return resp.status == 200
        else:
            universal_print(f'{params}')


def about_control():
    universal_print('=== about control ===')
    universal_print(f' ctl \tman_ctl')
    universal_print(f'  w  \t  i')
    universal_print(f'a s d\tj k l')
    universal_print()
    universal_print('u - upd and print sensors')
    universal_print('. - repeat last cmd')
    universal_print('q - quit')
    universal_print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--regime', nargs='?', default='real', help='regime: real/sim. Default: real')
    parser.add_argument('--robot-id', nargs='?', default='854CAF96103A6853', help='robot id')
    parser.add_argument('--ip', nargs='?', default='192.168.68.167', help='robot id')
    parser.add_argument('--ignore-cmd', dest='ignore_cmd', action='store_true', help='ignore cmd execution')
    parser.add_argument('--debug', dest='debug', action='store_true', help='debug output')
    args = parser.parse_args()

    regime = args.regime
    execute_cmd = not args.ignore_cmd
    robot_id = args.robot_id
    ip = args.ip
    debug = args.debug

    # if robot_id is None or ip is None:
    #     universal_print(f'Program is not started.')
    #     universal_print(f'robot_id and ip must be specified')
    #     exit(-1)

    universal_print(f'regime: {regime}')
    universal_print(f'execute cmd: {execute_cmd}')
    if regime == 'real':
        rc = RealRobotController(ip, robot_id)
    elif regime == 'sim':
        rc = SimRobotController('127.0.0.1:8801', '12345', pwm_time=500)
    else:
        universal_print(f'No such regime {regime}. Use real or sim')
        exit(0)

    about_control()
    rc.about()
    finished = False
    rc.execute_cmd = execute_cmd
    last_cmd = None

    while not finished:
        cmd = input('Command to perform: ')
        if cmd == '.' and last_cmd is not None:
            cmd = last_cmd
        else:
            last_cmd = cmd

        if cmd == 'w':
            universal_print(f'fwd 100 mm')
            rc.send_command_mv('f', 135)
        elif cmd == 'a':
            universal_print(f'left 90 grad')
            rc.send_command_mv('l', 90)
        elif cmd == 'd':
            universal_print(f'right 90 grad')
            rc.send_command_mv('r', 90)
        elif cmd == 's':
            universal_print(f'back 100 mm')
            rc.send_command_mv('b', 100)
        elif cmd == 'u':
            universal_print(f'upd sens')
            res = rc.request_sensor_data()
            rc.print_sensor_data()


        elif cmd == 'i':
            universal_print(f'manual fwd: {rc.pwm_val} for {rc.pwm_time} ms')
            rc.send_command_motor(rc.pwm_val, rc.pwm_val, rc.pwm_time, rc.pwm_time)
        elif cmd == 'j':
            universal_print(f'manual left: {rc.pwm_val} for {rc.pwm_time} ms')
            rc.send_command_motor(-rc.pwm_val, rc.pwm_val, rc.pwm_time, rc.pwm_time)
        elif cmd == 'l':
            universal_print(f'manual right: {rc.pwm_val} for {rc.pwm_time} ms')
            rc.send_command_motor(rc.pwm_val, -rc.pwm_val, rc.pwm_time, rc.pwm_time)
        elif cmd == 'k':
            universal_print(f'manual back: {rc.pwm_val} for {rc.pwm_time} ms')
            rc.send_command_motor(-rc.pwm_val, -rc.pwm_val, rc.pwm_time, rc.pwm_time)

        elif cmd == 'q':
            finished = True
            universal_print(f'Bye')
        else:
            universal_print(f'No cmd {cmd}')
