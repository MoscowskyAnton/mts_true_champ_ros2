import math

import rclpy
from rclpy.node import Node

import numpy as np

# 1. ranges
# get /mtc/driver/sensors and /mtc/slam/odom -
# post Marker with ranges from robot center

# 2. odom
# get /mtc/slam/proc_cmd and argrgate it
# post Odometry and Path

from mtc_msgs.msg import DirCommand,Sensors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

def angle_sum(a1, a2):
    diff = a1 + a2
    return (diff + math.pi) % (2*math.pi) - math.pi

class BagVisualizator(Node):

    def __init__(self):
        super().__init__('bag_vis_node')
        # constants
        self.length_coeff = 1.  # coefficient between commands and real linear motion
        init_pos_x = 0.09
        init_pos_y = 0.09

        # state variables:
        # 1. Calculated position
        self.curr_x = init_pos_x
        self.curr_y = init_pos_y
        self.curr_ori = math.pi * 0.5
        # 2. Odometry position
        self.odom_x = init_pos_x
        self.odom_y = init_pos_y
        self.odom_ori = math.pi * 0.5

        # ROS infrastructure
        # output messages
        self.msg_odom = Odometry()
        self.msg_odom.header.frame_id = 'map'
        self.msg_odom.pose.pose.orientation.w = 1.
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        #init_pose.header.stamp = self.get_clock().now()
        init_pose.pose.position.x = self.curr_x
        init_pose.pose.position.y = self.curr_y
        init_pose.pose.orientation.z = math.sin(self.curr_ori * 0.5)
        init_pose.pose.orientation.w = math.cos(self.curr_ori * 0.5)
        self.msg_path = Path()
        self.msg_path.header.frame_id = 'map'
        self.msg_path.poses = [init_pose]
        # subscriber for commands and publisher for odometry computed by these commands
        self.sub_proc_cmd = self.create_subscription(DirCommand, "slam/proc_cmd", self.cb_proc_cmd, 1)
        self.pub_odom = self.create_publisher(Odometry, '~/odom_by_commands', 1)
        self.pub_path = self.create_publisher(Path, '~/path_by_commands', 1)

        # subscriber for sensor odometry and publisher for sensors
        self.sub_slam_odom = self.create_subscription(Odometry, "slam/odom", self.cb_slam_odom, 1)
        self.sub_sensors = self.create_subscription(Sensors, "driver/sensors", self.cb_sensors, 1)
        #self.sub_driver_sensors = self.create_subscription(Odometry, "driver/sensors", self.cb_driver_sensors, 1)
        self.pub_sensors_from_odom = self.create_publisher(MarkerArray, '~/sensors_from_odom', 1)

    def cb_slam_odom(self, msg):
        # read and save odometry
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_ori = 2 * math.atan2(msg.pose.pose.orientation.z,
                                       msg.pose.pose.orientation.w)

    def cb_sensors(self, msg):
        #self.get_logger().info(f'message: {msg}')
        marker_size = 0.02
        # create output message
        out_array = MarkerArray()
        for i,(rd,ra) in enumerate(zip(msg.rangefinders_distances, msg.rangefinders_angles)):
            point_angle = angle_sum(self.odom_ori, ra)
            out_marker = Marker()
            out_marker.header.frame_id = 'map'
            out_marker.id = i
            out_marker.type = 2
            out_marker.action = 0
            out_marker.pose.position.x = self.odom_x + rd * math.cos(point_angle)
            out_marker.pose.position.y = self.odom_y + rd * math.sin(point_angle)
            #self.get_logger().info(f'calc: {i} {self.odom_ori} {ra} {rd} {out_marker.pose.position.x} {out_marker.pose.position.y}')
            out_marker.pose.orientation.w = 1.
            out_marker.scale.x = marker_size
            out_marker.scale.y = marker_size
            out_marker.scale.z = marker_size
            out_marker.color.r = 1.
            out_marker.color.g = 1.
            out_marker.color.b = 1.
            out_marker.color.a = 1.
            out_array.markers.append(out_marker)
        self.pub_sensors_from_odom.publish(out_array)

    def cb_proc_cmd(self, msg):
        # change position after command
        if msg.direction in [0,1]:
            # move forward or backward
            curr_dir_vector_x = math.cos(self.curr_ori)
            curr_dir_vector_y = math.sin(self.curr_ori)
            direction_sign = 1. if msg.direction == 0 else -1.
            new_x = self.curr_x + direction_sign * curr_dir_vector_x * msg.value * self.length_coeff
            new_y = self.curr_y + direction_sign * curr_dir_vector_y * msg.value * self.length_coeff
            self.curr_x = new_x
            self.curr_y = new_y

        else:
            # rotate to the left or to the right
            direction_sign = 1. if msg.direction == 2 else -1.
            new_ori = angle_sum(self.curr_ori, direction_sign * msg.value)
            #self.get_logger().info(f'value {msg.value}, dir {msg.direction}, new ori {new_ori}')
            self.curr_ori = new_ori

        #self.get_logger().info(f'after command {msg.direction} {msg.value} new position is {self.curr_x},{self.curr_y}, ori is {self.curr_ori}')

        # prepare messages
        #curr_time = self.get_clock().now()
        #self.msg_odom.header.stamp = curr_time
        self.msg_odom.pose.pose.position.x = self.curr_x
        self.msg_odom.pose.pose.position.y = self.curr_y
        self.msg_odom.pose.pose.orientation.z = math.sin(self.curr_ori * 0.5)
        self.msg_odom.pose.pose.orientation.w = math.cos(self.curr_ori * 0.5)
        new_pose = PoseStamped()
        new_pose.header.frame_id = 'map'
        #new_pose.header.stamp = curr_time
        new_pose.pose.position.x = self.curr_x
        new_pose.pose.position.y = self.curr_y
        new_pose.pose.orientation.z = math.sin(self.curr_ori * 0.5)
        new_pose.pose.orientation.w = math.cos(self.curr_ori * 0.5)
        #self.msg_path.header.stamp = curr_time
        self.msg_path.poses.append(new_pose)
        # publish messages
        self.pub_odom.publish(self.msg_odom)
        self.pub_path.publish(self.msg_path)


def main(args=None):
    rclpy.init(args=args)

    bag_vis = BagVisualizator()
    rclpy.spin(bag_vis)
    bag_vis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
