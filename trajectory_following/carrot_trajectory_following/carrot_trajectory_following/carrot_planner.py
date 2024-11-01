import math
from typing import List, Callable, Tuple, Optional

import numpy as np

from rclpy.node import Node
from mtc_msgs.srv import StepSLAMCommand
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovariance


def norm_angle(value: float) -> float:
    """
    To [-pi;pi]
    """
    return (value + np.pi) % (2 * np.pi) - np.pi


def euler_from_quaternion(
        quaternion: geometry_msgs.msg.Quaternion,
) -> Tuple[float, float, float]:
    """
    Converts quaternion to euler roll, pitch, yaw.
    When ROS 2 Python tf_conversions becomes available, replace this code with call to the library.

    :returns : Tuple[roll, pitch, yaw]
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


class CarrotPose:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self):
        return f"Pose({self.x:.2}, {self.y:.2})"


def get_2d_pose_from_3d(pose_3d: geometry_msgs.msg.Pose) -> CarrotPose:
    x = pose_3d.position.x
    y = pose_3d.position.y
    roll, pitch, yaw = euler_from_quaternion(pose_3d.orientation)

    return CarrotPose(x, y, yaw)


class TrajectoryFollowing:
    """
    While trajectory following, generally speaking, should use a map to avoid obstacles,
    Carrot planner doesn't, so superclass shouldn't require the map either.

    :param send_cmd_and_get_sensors: function that sends a movement command to the robot and gets sensor data back
        after the execution (Odometry in this case)
    :param distance_tolerance: m, distance to goal tolerance
    :param angle_tolerance: m, angle difference to goal tolerance during movement AND in the ending pose
    :param min_cmd_dist: m, min distance that can be issued as a command to move (or control may be too imprecise)
    :param max_cmd_dist: m, max distance that can be issued as a command to move (since localization may break otherwise)
    :param min_cmd_angle: rad, min angle that can be issued as a command to move (or control may be too imprecise)
    :param max_cmd_angle: rad, max angle that can be issued as a command to move (since localization may break otherwise)
    """

    def __init__(
            self,
            # DEBUG
            ros_node: Node,

            send_cmd_and_get_sensors: Callable[[np.uint8, float], PoseWithCovariance],
            distance_tolerance: float = 0.1,  # m
            angle_tolerance: float = 0.17,  # rad, ~10 degrees
            min_cmd_dist: float = 0.1,  # m
            max_cmd_dist: float = 0.3,  # m
            min_cmd_angle: float = 0.35,  # rad, ~20 degrees
            max_cmd_angle: float = 1.57,  # rad, ~90 degrees
    ):
        self.node = ros_node
        # connect to cmd messages function (should come from ROS service)
        self.send_cmd_and_get_sensors = send_cmd_and_get_sensors

        # for pose comparison
        self.distance_tolerance = distance_tolerance  # m
        self.angle_tolerance = angle_tolerance  # rad, ~10 degrees

        # command limits
        self.min_cmd_dist = min_cmd_dist
        self.max_cmd_dist = max_cmd_dist
        self.min_cmd_angle = min_cmd_angle
        self.max_cmd_angle = max_cmd_angle

        assert self.min_cmd_angle < self.angle_tolerance, (
            f"Goal angle tolerance {self.angle_tolerance} should be bigger than minimum possible"
            f" turn {self.min_cmd_angle} to avoid infinite correcting movement (hardware too imprecise)"
        )

        assert self.min_cmd_dist < self.distance_tolerance, (
            f"Goal distance tolerance {self.distance_tolerance} should be bigger than minimum possible "
            f" command {self.min_cmd_dist} to avoid infinite correcting movement (hardware too imprecise)"
        )

        # path to follow
        self.target_path = []
        # traversed path for debug
        self.traversed_path = []

    def execute_plan(self, target_path: List[CarrotPose]):
        """
        Follows the given path.
        """
        raise NotImplementedError()

    def is_task_finished(self) -> bool:
        """
        Checks if the problem task was completed successfully.
        """
        raise NotImplementedError()


class CarrotPlanner(TrajectoryFollowing):
    def __init__(
            self,
            # DEBUG
            ros_node: Node,

            send_cmd_and_get_sensors: Callable[[np.uint8, float], PoseWithCovariance],
            distance_tolerance: float = 0.1,  # m
            angle_tolerance: float = 0.17,  # rad, ~10 degrees
            min_cmd_dist: float = 0.1,  # m
            max_cmd_dist: float = 0.3,  # m
            min_cmd_angle: float = 0.35,  # rad, ~20 degrees
            max_cmd_angle: float = 1.57,  # rad, ~90 degrees
    ):
        """
        Goes directly to the next point in the path, ignoring all obstacles. Executes successive turns and forward
        movements to achieve it.

        NOTE: this planner ignores pose orientations in the path and just follows the coordinates.

        :param send_cmd_and_get_sensors: method that can be used to send movement commands to the robot
        """
        super().__init__(
            ros_node,
            send_cmd_and_get_sensors,
            distance_tolerance,
            angle_tolerance,
            min_cmd_dist,
            max_cmd_dist,
            min_cmd_angle,
            max_cmd_angle,
        )

        self.latest_pose: Optional[CarrotPose] = None

    def execute_plan(self, target_path: List[CarrotPose]):
        """
        Follows the given path.
        """
        self.node.get_logger().info(f"Executing plan in carrot planner")
        # initialize sensor data by sending an empty movement command and getting back the data
        latest_pose_covar = self.send_cmd_and_get_sensors(
            StepSLAMCommand.Request.LEFT, 0.0
        )
        self.node.get_logger().info(f"Got latest_pose_covar from cmd_and_sensors")
        self.latest_pose = get_2d_pose_from_3d(latest_pose_covar.pose)

        self.traversed_path.clear()
        # part of the plan path still not traversed
        current_plan_idx = 0
        while current_plan_idx < len(target_path):
            self.node.get_logger().info(
                f"[{current_plan_idx + 1}/{len(target_path)}] moving from pose {self.latest_pose}"
                f" to {target_path[current_plan_idx]}")

            self.traversed_path.append(self.latest_pose)
            self.move_to_pose(target_path[current_plan_idx])
            current_plan_idx += 1

        self.node.get_logger().info(f"Finished executing plan")

    def is_task_finished(self) -> bool:
        """
        Checks if the problem task was completed successfully.
        """
        # if last pose of the plan is reached, we are good
        return self.close_enough(self.latest_pose, self.target_path[-1])

    def close_enough(self, pose1: CarrotPose, pose2: CarrotPose) -> bool:
        """
        Checks if pose is close enough
        """
        pose1_vec = np.array((pose1.x, pose1.y))
        pose2_vec = np.array((pose2.x, pose2.y))

        dist = np.linalg.norm((pose1_vec - pose2_vec))
        dangle = norm_angle(pose1.theta - pose2.theta)

        return dist < self.distance_tolerance and abs(dangle) < self.angle_tolerance

    def move_to_pose(self, target_pose: CarrotPose) -> None:
        """
        Move to the next plan path element (next pose), which SHOULD be adjacent
        to the current cell (or at least with no obstacles between the poses).
        """
        while True:
            # NOTE: latest odom should have been initialized by an empty command-get-sensors by now
            dx_to_target = target_pose.x - self.latest_pose.x
            dy_to_target = target_pose.y - self.latest_pose.y
            angle_to_target = np.arctan2(dy_to_target, dx_to_target)

            # how different is the orientation from direction to the target
            dangle_to_target = norm_angle(angle_to_target - self.latest_pose.theta)
            abs_dangle_to_target = abs(dangle_to_target)
            # NEGATIVE when LEFT turn needed, POSITIVE when RIGHT turn needed
            if dangle_to_target >= 0:
                required_turn = StepSLAMCommand.Request.LEFT
            else:
                required_turn = StepSLAMCommand.Request.RIGHT

            # how far from the target
            distance_to_target = np.hypot(dx_to_target, dy_to_target)

            # goal reached, avoid checking angle error
            if distance_to_target < self.distance_tolerance:
                return

            if abs_dangle_to_target > self.angle_tolerance:
                # turn to goal
                # apply limits
                angle_dist = abs_dangle_to_target
                angle_dist = min(self.max_cmd_angle, angle_dist)
                angle_dist = max(self.min_cmd_angle, angle_dist)

                self.node.get_logger().info(
                    f"fixing angle error: from {math.degrees(self.latest_pose.theta):.0f}"
                    f" to {math.degrees(angle_to_target):.0f} by {math.degrees(dangle_to_target):.0f}")

                # NOTE: this is expected to be a blocking call that waits until command completion
                latest_pose_covar = self.send_cmd_and_get_sensors(
                    required_turn, float(angle_dist)
                )
                self.latest_pose = get_2d_pose_from_3d(latest_pose_covar.pose)

                # reestimate pose and relation to goal
                continue

            # we should look at the goal at this point, more or less
            if distance_to_target > self.distance_tolerance:
                # move forward to goal
                # apply limits
                move_dist = distance_to_target
                move_dist = min(self.max_cmd_dist, move_dist)
                move_dist = max(self.min_cmd_dist, move_dist)

                self.node.get_logger().info(
                    f"fixing distance error: {distance_to_target} by {move_dist}")

                # NOTE: should be a blocking call that waits until command completion
                latest_pose_covar = self.send_cmd_and_get_sensors(
                    StepSLAMCommand.Request.FORWARD, float(move_dist)
                )
                self.latest_pose = get_2d_pose_from_3d(latest_pose_covar.pose)

                # reestimate pose and relation to goal
                continue

# makes only 2 steps: rotate and move
class CarrotPlannerLite(CarrotPlanner):

    def move_to_pose(self, target_pose: CarrotPose) -> None:
        """
        Move to the next plan path element (next pose), which SHOULD be adjacent
        to the current cell (or at least with no obstacles between the poses).
        """
        #while True:
        for step in ['rotate', 'move']:
            # NOTE: latest odom should have been initialized by an empty command-get-sensors by now
            dx_to_target = target_pose.x - self.latest_pose.x
            dy_to_target = target_pose.y - self.latest_pose.y
            angle_to_target = np.arctan2(dy_to_target, dx_to_target)

            # how different is the orientation from direction to the target
            dangle_to_target = norm_angle(angle_to_target - self.latest_pose.theta)
            abs_dangle_to_target = abs(dangle_to_target)
            # NEGATIVE when LEFT turn needed, POSITIVE when RIGHT turn needed
            if dangle_to_target >= 0:
                required_turn = StepSLAMCommand.Request.LEFT
            else:
                required_turn = StepSLAMCommand.Request.RIGHT

            # how far from the target
            distance_to_target = np.hypot(dx_to_target, dy_to_target)

            # goal reached, avoid checking angle error
            if distance_to_target < self.distance_tolerance:
                return

            if step == 'rotate':
                if abs_dangle_to_target > self.angle_tolerance:
                    # turn to goal
                    # apply limits
                    angle_dist = abs_dangle_to_target
                    angle_dist = min(self.max_cmd_angle, angle_dist)
                    angle_dist = max(self.min_cmd_angle, angle_dist)

                    self.node.get_logger().info(
                        f"fixing angle error: from {math.degrees(self.latest_pose.theta):.0f}"
                        f" to {math.degrees(angle_to_target):.0f} by {math.degrees(dangle_to_target):.0f}")

                    # NOTE: this is expected to be a blocking call that waits until command completion
                    latest_pose_covar = self.send_cmd_and_get_sensors(
                        required_turn, float(angle_dist)
                    )
                    self.latest_pose = get_2d_pose_from_3d(latest_pose_covar.pose)

                    # reestimate pose and relation to goal
                    continue

            if step == 'move':
                # we should look at the goal at this point, more or less
                if distance_to_target > self.distance_tolerance:
                    # move forward to goal
                    # apply limits
                    move_dist = distance_to_target
                    move_dist = min(self.max_cmd_dist, move_dist)
                    move_dist = max(self.min_cmd_dist, move_dist)

                    self.node.get_logger().info(
                        f"fixing distance error: {distance_to_target} by {move_dist}")

                    # NOTE: should be a blocking call that waits until command completion
                    latest_pose_covar = self.send_cmd_and_get_sensors(
                        StepSLAMCommand.Request.FORWARD, float(move_dist)
                    )
                    self.latest_pose = get_2d_pose_from_3d(latest_pose_covar.pose)

                    # reestimate pose and relation to goal
                    continue
