import numpy as np
import rclpy
from mtc_msgs.msg import Sensors
from mtc_msgs.srv import GetSensors, DirectionCommand
from rclpy.node import Node
from std_msgs.msg import String

from mtc_drivers.robot_driver_lib import RealRobotController


class RealDriver(Node):
    def __init__(self):

        super().__init__('driver')

        self.get_logger().info("Initializing RealDriver node")

        self.declare_parameter('robot_id', "854CAF96103A6853")
        self.declare_parameter('ip', "192.168.68.167")
        self.declare_parameter('sensor_timer_period_s', 0.1)

        self.debug_directions = True
        self.debug_commands = True

        self.robot_x = 88.
        self.robot_y = 70.
        self.robot_y_platfrom = 60.

        self.declare_parameter('max_r', 2.0)
        self.max_r = self.get_parameter('max_r').value

        self.sensors_pub = self.create_publisher(Sensors, '~/sensors', 10)
        self.debug_commands_pub = self.create_publisher(String, '~/debug_commands', 10)

        self.RRC = RealRobotController(
            node=self,
            ip_=self.get_parameter('ip').value,
            robot_id_=self.get_parameter('robot_id').value,
            event_registrator=None
        )

        self.sensors_srv = self.create_service(GetSensors, '~/get_sensors', self.get_sensors_cb)
        self.direction_cmd_srv = self.create_service(DirectionCommand, 'dir_command', self.direction_cmd_cb)

        self.sensors_timer = self.create_timer(self.get_parameter('sensor_timer_period_s').value, self.sensors_timer_callback)

    def add_debug_data(self, debug_str):
        # send info about last command in topic
        msg = String()
        msg.data = debug_str
        self.debug_commands_pub.publish(msg)

    def sens_upd(self, sens_msg):
        sens_msg.rangefinders_distances = [0.0] * 6
        sens_msg.rangefinders_angles = [0.0] * 6

        sens_msg.rangefinders_distances[0] = min(float(self.RRC.laser['f'] + self.robot_x / 2) / 1000., self.max_r)
        sens_msg.rangefinders_angles[0] = 0.0

        sens_msg.rangefinders_distances[1] = min(float(self.RRC.laser['l'] + self.robot_y / 2) / 1000., self.max_r)
        sens_msg.rangefinders_angles[1] = np.pi / 4

        sens_msg.rangefinders_distances[2] = min(
            float(self.RRC.laser['sl'] + np.hypot(self.robot_x / 2, self.robot_y_platfrom / 2)) / 1000., self.max_r)
        sens_msg.rangefinders_angles[2] = np.pi / 2

        sens_msg.rangefinders_distances[3] = min(float(self.RRC.laser['b'] + self.robot_x / 2) / 1000., self.max_r)
        sens_msg.rangefinders_angles[3] = np.pi

        sens_msg.rangefinders_distances[4] = min(
            float(self.RRC.laser['sr'] + np.hypot(self.robot_x / 2, self.robot_y_platfrom / 2)) / 1000., self.max_r)
        sens_msg.rangefinders_angles[4] = -np.pi / 2

        sens_msg.rangefinders_distances[5] = min(float(self.RRC.laser['r'] + self.robot_y / 2) / 1000., self.max_r)
        sens_msg.rangefinders_angles[5] = -np.pi / 4



        sens_msg.robot_yaw = np.deg2rad(float(self.RRC.imu['y']))

        return sens_msg

    def sensors_timer_callback(self):
        now = self.get_clock().now()
        status, _, parsed = self.RRC.request_sensor_data()
        #self.get_logger().info(f"Sensors parsed: {parsed}")
        if status:
            # base sensors msg
            sens_msg = Sensors()
            sens_msg.header.stamp = now.to_msg()

            sens_msg = self.sens_upd(sens_msg)

            self.sensors_pub.publish(sens_msg)

    def get_sensors_cb(self, request, responce):

        for _ in range(6): # because sensors are too slow
            status, _, parsed = self.RRC.request_sensor_data()
            #self.get_logger().info(f"Sensors parsed: {parsed}")

        responce.success = status
        if status:
            responce = self.sens_upd(responce)

        return responce

    def direction_cmd_cb(self, request, responce):
        direction = ""
        value = 0
        if request.direction == request.FORWARD:
            direction = 'f'
            value = int(request.value * 1000)
        elif request.direction == request.BACKWARD:
            direction = 'b'
            value = int(request.value * 1000)
        elif request.direction == request.LEFT:
            direction = 'l'
            value = int(np.rad2deg(request.value))
        elif request.direction == request.RIGHT:
            direction = 'r'
            value = int(np.rad2deg(request.value))

        responce.success = self.RRC.send_command_mv(direction, value)
        return responce


def main(args=None):
    rclpy.init(args=args)

    driver = RealDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
