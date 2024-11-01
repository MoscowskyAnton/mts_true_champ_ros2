import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from mtc_msgs.srv import FollowTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class RvizSubNode3(Node):

    def __init__(self):
        super().__init__('rviz2path')
        self.cmd_client = self.create_client(
            FollowTrajectory, "/mtc/follow_trajectory_srv"
        )
        self.point_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.client_futures = []

    def goal_cb(self, msg):
        #print("sub_cb({})".format(msg.data))
        request = FollowTrajectory.Request()
        request.goal_path.poses.clear()
        request.goal_path.poses.append(msg)
        self.client_futures.append(self.cmd_client.call_async(request))

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

class RvizSubNode2(Node):
    def __init__(self):
        super().__init__("rviz2path")

        self.cmd_client = self.create_client(
            FollowTrajectory, "/mtc/follow_trajectory_srv"
        )

        self.get_logger().info("Starting rviz2path service")
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for trajectory service")
        self.cmd_request = FollowTrajectory.Request()

        self.point_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

    def _send_cmd_request(self, path_: Path):
        self.cmd_request.goal_path = path_
        return self.cmd_client.call_async(self.cmd_request)

    def send_cmd_blocking(self, path_: Path):
        future = self._send_cmd_request(path_)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"cmd status = {response.status}")

    def goal_cb(self, msg):
        self.get_logger().info("Sending rviz2path command")

        path = Path()
        path.poses.clear()
        path.poses.append(msg)
        self.send_cmd_blocking(path)


class RvizSubNode(Node):
    def __init__(self):
        super().__init__("rviz2path")

        self.cmd_client = self.create_client(
            FollowTrajectory, "/mtc/follow_trajectory_srv"
        )

        self.get_logger().info("Starting rviz2path service")
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for trajectory service")
        self.cmd_request = FollowTrajectory.Request()

        self.point_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

    def goal_cb(self, msg):
        self.get_logger().info("Sending rviz2path command")
        self.cmd_request.goal_path.poses.clear()
        self.cmd_request.goal_path.poses.append(msg)

        future = self.cmd_client.call_async(self.cmd_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"cmd status = {response.success}")


def main():
    rclpy.init()

    rsn = RvizSubNode3()
    rsn.spin()

    #rclpy.spin(rsn)
    #try:
        #executor = MultiThreadedExecutor()
        #rclpy.spin(rsn, executor)
    #except KeyboardInterrupt:
        #pass
    #finally:
        #rsn.destroy_node()
        #rclpy.shutdown()




if __name__ == "__main__":
    main()
