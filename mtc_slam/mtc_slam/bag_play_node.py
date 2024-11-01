import rclpy
from rclpy.node import Node

from mtc_msgs.msg import DirCommand, Sensors
from mtc_msgs.srv import GetSensors, StepSLAMCommand, DirectionCommand


class BagPlay(Node):

    def __init__(self):
        super().__init__('bag_play_node')

        self.sub_sensors = self.create_subscription(Sensors, "driver/sensors", self.cb_sensors, 1)

        self.sub_commands = self.create_subscription(DirCommand, "slam/proc_cmd", self.cb_commands, 1)

        self.sensors_srv = self.create_service(GetSensors, '~/get_sensors', self.get_sensors_cb)

        # self.direction_cmd_srv = self.create_service(DirectionCommand, 'dir_command', self.direction_cmd_cb)

        self.client_futures = []
        self.last_sensors = None

        self.slam_cmd_cli = self.create_client(StepSLAMCommand, 'slam/cmd')
        while not self.slam_cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def cb_sensors(self, msg):
        self.last_sensors = msg

    def cb_commands(self, msg):
        self.get_logger().info(f"Get cmd, cashing request...")
        request = StepSLAMCommand.Request()
        request.direction = msg.direction
        request.value = msg.value
        self.client_futures.append(self.slam_cmd_cli.call_async(request))

    def get_sensors_cb(self, request, response):
        if not self.last_sensors is None:
            response.rangefinders_distances = self.last_sensors.rangefinders_distances
            response.rangefinders_angles = self.last_sensors.rangefinders_angles
            response.robot_yaw = self.last_sensors.robot_yaw
            response.success = True
        return response

    def direction_cmd_cb(self, request, response):
        # do nothing
        for _ in range(100):
            self.create_rate(0.02).sleep()
            rclpy.spin_once()

        return response

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    print("received service result: {}".format(res))
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures

def main():
    rclpy.init()

    BagPlay().spin()


if __name__ == "__main__":
    main()
