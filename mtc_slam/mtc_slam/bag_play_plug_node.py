import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from mtc_msgs.srv import DirectionCommand

class BagPlug(Node):

    def __init__(self):
        super().__init__('bag_plug_node')

        self.declare_parameter('wait_time', 1.)

        self.loop_rate = self.create_rate(1 / self.get_parameter("wait_time").value, self.get_clock())

        self.direction_cmd_srv = self.create_service(DirectionCommand, 'dir_command', self.direction_cmd_cb)

    def direction_cmd_cb(self, request, response):
        # do nothing except sleep
        self.loop_rate.sleep()

        return response


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    plug = BagPlug()
    rclpy.spin(plug, executor=executor)


if __name__ == "__main__":
    main()
