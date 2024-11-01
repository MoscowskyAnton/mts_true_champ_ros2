from typing import List, Callable, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from mtc_msgs.srv import FollowTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path


class ExamplePathNode(Node):
    def __init__(self):
        super().__init__("example_path")

        self.cmd_client = self.create_client(
            FollowTrajectory, "/mtc/follow_trajectory_srv"
        )
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for trajectory service")
        self.cmd_request = FollowTrajectory.Request()

    def _send_cmd_request(self, path_: Path):
        self.cmd_request.goal_path = path_
        return self.cmd_client.call_async(self.cmd_request)

    def send_cmd_blocking(self, path_: Path):
        future = self._send_cmd_request(path_)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"cmd status = {response.status}")


def main():
    rclpy.init()

    example_node = ExamplePathNode()

    #goal_poses = [(3.0, 3.0, 0.5), (4.0, 5.0, 0.0)]
    goal_poses = [(0.0, 0.0, 0.0),]

    path = Path()

    for pose2d in goal_poses:
        pose_stamped = PoseStamped()
        pose_stamped.pose = Pose()
        pose_stamped.pose.position.x = pose2d[0]
        pose_stamped.pose.position.y = pose2d[1]

        # orientation and theta is ignored cause quaternions
        path.poses.append(pose_stamped)

    example_node.send_cmd_blocking(path)

    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
