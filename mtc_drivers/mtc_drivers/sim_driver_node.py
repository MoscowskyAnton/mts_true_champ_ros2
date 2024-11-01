import rclpy
from rclpy.node import Node

import http.client
import json

from mtc_msgs.msg import Sensors, SetPWM
from mtc_msgs.srv import GetSensors
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def norm_angle(value):
    return (value + np.pi) % (2*np.pi) - np.pi

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class SimulatorDriver(Node):
    def __init__(self):
        super().__init__('driver')

        self.token = '63509e8c-4687-45ae-80fb-13638eb513ad4206e9e5-68fb-41fc-b5a1-bccc1e2b010f'

        self.robot_x = 65
        self.robot_y = 69

        self.declare_parameter('sensor_timer_period_s', 0.05)
        self.declare_parameter('command_timer_period_s', 0.1)
        self.declare_parameter('stop_time', 0.5)

        self.sensors_pub = self.create_publisher(Sensors, '~/sensors', 10)

        self.true_odom_pub = self.create_publisher(Odometry, '~/true_odom', 1)

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', -np.pi/2)

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.start_x, self.start_y, self.start_yaw = None, None, None

        self.declare_parameter('restart_maze_on_start', True)
        if self.get_parameter('restart_maze_on_start'):
            self.restart_maze()

        self.prev_odom = None
        
        self.declare_parameter('ranges_sigma', 0.0)        
        

        self.sensors_timer = self.create_timer(self.get_parameter('sensor_timer_period_s').value, self.sensors_timer_callback)

        self.pwm_timer = self.create_timer(self.get_parameter('command_timer_period_s').value, self.pwm_timer_cb)

        self.current_cmd = None#(0, 0)
        self.last_cmd_time = None
        self.pwm_sub = self.create_subscription(
            SetPWM,
            '~/pwm',
            self.pwm_cb,
            10)
        self.pwm_sub  # prevent unused variable warning

        self.sensors_srv = self.create_service(GetSensors, '~/get_sensors', self.get_sensosrs_cb)

    def pwm_cb(self, msg):
        if msg.time != 0:
            self.current_cmd = None
            self.set_command(msg.left_pwm, msg.right_pwm, msg.time)
        else:
            self.current_cmd = (msg.left_pwm, msg.right_pwm)
            self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def pwm_timer_cb(self):
        if self.current_cmd is None:
            return
        if (self.get_clock().now().nanoseconds * 1e-9 - self.last_cmd_time) > self.get_parameter('stop_time').value:
            self.set_command(0, 0, self.get_parameter('command_timer_period_s').value)
            return

        self.set_command(self.current_cmd[0], self.current_cmd[1], self.get_parameter('command_timer_period_s').value)

    def get_sensosrs_cb(self, request, response):
        response.success, sensors = self.get_sensors()
        if response.success:
            response.rangefinders_distances = [0.0] * 6
            response.rangefinders_angles = [0.0] * 6

            response.rangefinders_distances[0] = (sensors['front_distance'] + self.robot_x/2) / 100
            response.rangefinders_angles[0] = 0
            response.rangefinders_distances[0] += np.random.normal(0, self.get_parameter('ranges_sigma').value)

            response.rangefinders_distances[1] = (sensors['left_45_distance'] + np.hypot(self.robot_x /2, self.robot_y /2))/100
            response.rangefinders_angles[1] = np.pi/4
            response.rangefinders_distances[1] += np.random.normal(0, self.get_parameter('ranges_sigma').value)

            response.rangefinders_distances[2] = (sensors['left_side_distance'] + self.robot_y/2)/100
            response.rangefinders_angles[2] = np.pi/2
            response.rangefinders_distances[2] += np.random.normal(0, self.get_parameter('ranges_sigma').value)

            response.rangefinders_distances[3] = (sensors['back_distance'] + self.robot_x/2)/100
            response.rangefinders_angles[3] = np.pi
            response.rangefinders_distances[3] += np.random.normal(0, self.get_parameter('ranges_sigma').value)

            response.rangefinders_distances[4] = (sensors['right_side_distance'] + self.robot_y/2)/100
            response.rangefinders_angles[4] = -np.pi/2
            response.rangefinders_distances[4] += np.random.normal(0, self.get_parameter('ranges_sigma').value)

            response.rangefinders_distances[5] = (sensors['right_45_distance'] + np.hypot(self.robot_x /2, self.robot_y /2))/100
            response.rangefinders_angles[5] = -np.pi/4
            response.rangefinders_distances[5] += np.random.normal(0, self.get_parameter('ranges_sigma').value)
                                                                   

            response.robot_yaw = norm_angle(np.deg2rad(sensors['rotation_yaw']))# + np.pi/2)

        return response


    def sensors_timer_callback(self):
        now = self.get_clock().now()
        status, sensors = self.get_sensors()
        if status:
            # base sensors msg
            sens_msg = Sensors()
            sens_msg.header.stamp = now.to_msg()

            sens_msg.rangefinders_distances = [0.0] * 6
            sens_msg.rangefinders_angles = [0.0] * 6

            sens_msg.rangefinders_distances[0] = (sensors['front_distance'] + self.robot_x/2) / 100
            sens_msg.rangefinders_angles[0] = 0

            sens_msg.rangefinders_distances[1] = (sensors['left_45_distance'] + np.hypot(self.robot_x /2, self.robot_y /2))/100
            sens_msg.rangefinders_angles[1] = np.pi/4

            sens_msg.rangefinders_distances[2] = (sensors['left_side_distance'] + self.robot_y/2)/100
            sens_msg.rangefinders_angles[2] = np.pi/2

            sens_msg.rangefinders_distances[3] = (sensors['back_distance'] + self.robot_x/2)/100
            sens_msg.rangefinders_angles[3] = np.pi

            sens_msg.rangefinders_distances[4] = (sensors['right_side_distance'] + self.robot_y/2)/100
            sens_msg.rangefinders_angles[4] = -np.pi/2

            sens_msg.rangefinders_distances[5] = (sensors['right_45_distance'] + np.hypot(self.robot_x /2, self.robot_y /2))/100
            sens_msg.rangefinders_angles[5] = -np.pi/4

            sens_msg.robot_yaw = norm_angle(np.deg2rad(sensors['rotation_yaw']))# + np.pi/2)

            self.sensors_pub.publish(sens_msg)

            # true odometry
            if self.start_x is None:
                self.start_x = sensors['down_y_offset']
                self.start_y = sensors['down_x_offset']
                self.start_yaw = np.deg2rad(sensors['rotation_yaw'])

            odom_msg = Odometry()

            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "true_odom"

            odom_msg.pose.pose.position.x = self.x + (sensors['down_y_offset'] - self.start_x)/100 # m
            odom_msg.pose.pose.position.y = self.y + (sensors['down_x_offset'] - self.start_y)/100 # m

            yaw = -(self.yaw + np.deg2rad(sensors['rotation_yaw']) - self.start_yaw)
            #yaw = np.deg2rad(sensors['rotation_yaw'])
            q = quaternion_from_euler(0, 0, yaw)
            odom_msg.pose.pose.orientation = Quaternion(x = q[1], y = q[2], z = q[3], w = q[0])

            now_ = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
            if self.prev_odom is None:
                v, w = 0.0, 0.0
            else:
                #dt = now.nanoseconds * 1e-9 - self.prev_odom[3]

                dt = now_ - self.prev_odom[3]
                #self.get_logger().info(f"{dt}")

                dx = odom_msg.pose.pose.position.x - self.prev_odom[0]
                dy = odom_msg.pose.pose.position.y - self.prev_odom[1]
                #dr = np.hypot(dx, dy) * np.sign(np.arctan2(dy, dx))
                dr = np.hypot(dx, dy)
                a = np.arctan2(dy, dx)
                if np.abs(norm_angle(a - yaw)) > 1.5:
                    dr = -dr
                dyaw = norm_angle(yaw - self.prev_odom[2])

                v, w = dr / dt, dyaw / dt

            #self.get_logger().info(f"{v} {w}")
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.prev_odom = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw, now_

            self.true_odom_pub.publish(odom_msg)


    '''
    {'front_distance': 49.48, 'right_side_distance': 382.14, 'left_side_distance': 59.33, 'back_distance': 49.72, 'left_45_distance': 69.75, 'right_45_distance': 70.75, 'rotation_pitch': -0.27, 'rotation_yaw': 0.0, 'rotation_roll': -0.13, 'down_x_offset': -1245.87, 'down_y_offset': -1254.1}
    '''
    def get_sensors(self):
        connection = http.client.HTTPConnection('127.0.0.1:8801')
        connection.request('GET', f'/api/v1/robot-cells/sensor-data?token={self.token}', headers={'accept': 'application/json'})
        response = connection.getresponse()
        if response.status != 200:
            self.get_logger().error(f'Error read {response.status} {response.read()}')
            return False, None
        else:
            parsed_resp = json.loads(response.read())
            return True, parsed_resp

    def set_command(self, pwm_l, pwm_r, time = 1):
        pwm_l = int(pwm_l)
        pwm_r = int(pwm_r)
        #print(f'set_command: {pwm_l} {pwm_r}')
        connection = http.client.HTTPConnection('127.0.0.1:8801')

        connection.request('POST', f'/api/v1/robot-motors/move?token={self.token}&l={pwm_l}&l_time={time}&r={pwm_r}&r_time={time}', headers={'accept': 'application/json'})

        response = connection.getresponse()
        if response.status != 200:
            #print(f'Error {response.status} {response.read()}')
            self.get_logger().error(f'Error send {response.status} {response.read()}')
            return False
        return True


    def restart_maze(self):
        connection = http.client.HTTPConnection('127.0.0.1:8801')
        connection.request('POST', f'/api/v1/maze/restart?token={self.token}', headers={'accept': 'application/json'})
        response = connection.getresponse()
        if response.status != 200:
            return False
        return True




def main(args=None):
    #print('Hi from mtc_drivers.')
    rclpy.init(args=args)

    driver = SimulatorDriver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
