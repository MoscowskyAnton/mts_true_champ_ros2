from threading import Event
from typing import Optional, List

import numpy as np
from carrot_trajectory_following.carrot_planner import (
    CarrotPlanner,
    CarrotPlannerLite,
    CarrotPose,
    get_2d_pose_from_3d
)

import rclpy
from mtc_msgs.srv import StepSLAMCommand, FollowTrajectory
from geometry_msgs.msg import PoseWithCovariance
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class CarrotNode(Node):
    def __init__(self):
        super().__init__("carrot_planner")

        # Init ROS2 interfaces
        self.latest_pose_covar: Optional[PoseWithCovariance] = None

        self.get_logger().info("Initializing CarrotNode")
        self.cmd_client = self.create_client(StepSLAMCommand, "step_slam_command")
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for command service")
        self.cmd_request = StepSLAMCommand.Request()

        # self.follow_trajectory_srv_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()
        self.follow_trajectory_srv = self.create_service(
            FollowTrajectory, "follow_trajectory_srv", self.follow_trajectory_cb, callback_group=self.callback_group
        )

        self.declare_parameter('distance_tolerance', 0.15)  # m
        self.declare_parameter('angle_tolerance', 0.52)  # rad, ~30 deg
        self.declare_parameter('min_cmd_dist', 0.1)  # m
        self.declare_parameter('max_cmd_dist', 0.9)  # m
        self.declare_parameter('min_cmd_angle', 0.35)  # rad, ~20 degrees
        self.declare_parameter('max_cmd_angle', 1.57)  # rad, ~90 degrees

        # init trajectory following and pass it the interfaces
        #self.trajectory_following = CarrotPlanner(
        self.trajectory_following = CarrotPlannerLite(
            self,
            self.send_cmd_and_get_sensors,
            distance_tolerance=self.get_parameter('distance_tolerance').value,
            angle_tolerance=self.get_parameter('angle_tolerance').value,
            min_cmd_dist=self.get_parameter('min_cmd_dist').value,  # m
            max_cmd_dist=self.get_parameter('max_cmd_dist').value,  # m
            min_cmd_angle=self.get_parameter('min_cmd_angle').value,  # rad
            max_cmd_angle=self.get_parameter('max_cmd_angle').value,  # rad
        )

        self.get_logger().info("Finished initializing CarrotNode")

    def get_current_pose(self) -> CarrotPose:
        return get_2d_pose_from_3d(self.latest_pose_covar.pose)

    def follow_trajectory_cb(self, request, response):
        # change ROS2 plan to a list of poses
        poses_path: List[CarrotPose] = []
        for stamped_pose in request.goal_path.poses:
            pose_2d = get_2d_pose_from_3d(stamped_pose.pose)
            poses_path.append(pose_2d)

        self.get_logger().info(f"Incoming request to follow trajectory: {poses_path}")
        self.trajectory_following.execute_plan(poses_path)
        self.get_logger().info(f"Finished following trajectory: {poses_path}")

        response.status = True

        return response

    def _send_cmd_request(self, direction_: np.uint8, value_: float):
        self.cmd_request.direction = direction_
        self.cmd_request.value = value_
        self.get_logger().info(f"sending request")
        return self.cmd_client.call_async(self.cmd_request)

    def send_cmd_and_get_sensors(self, direction_: np.uint8, value_: float) -> PoseWithCovariance:
        self.get_logger().info(f"cmd sent {direction_} {value_}")
        cmd_event = Event()

        def done_callback(futur, cmd_even):
            cmd_even.set()

        future = self._send_cmd_request(direction_, value_)
        future.add_done_callback(lambda f, e=cmd_event: done_callback(f, e))
        # future.add_done_callback(done_callback)
        # rclpy.spin_until_future_complete(self, future)
        cmd_event.wait()
        response = future.result()
        self.get_logger().info(f"cmd status = {response.success}")
        self.latest_pose_covar = response.pose
        return self.latest_pose_covar


def main():
    rclpy.init()

    planner_node = CarrotNode()
    planner_node.get_logger().info(f"started node")
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(planner_node, executor)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
