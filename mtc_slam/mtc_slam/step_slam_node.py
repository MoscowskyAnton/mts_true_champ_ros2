import rclpy
from rclpy.node import Node
from rclpy import time
import numpy as np
from mtc_msgs.msg import Sensors
from geometry_msgs.msg import Quaternion, PoseArray, Pose
from threading import Lock
import time as pytime
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from mtc_slam.slam_lib import SLAM, skukozjit_walls
from mtc_msgs.srv import StepSLAMCommand, DirectionCommand, GetSensors
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

#from rclpy.qos import qos_profile_system_default
#from rclpy.service_introspection import ServiceIntrospectionState


class StepSLAM(Node):
    
    def __init__(self):
        super().__init__('slam')
        
        self.declare_parameter('lab_size', [0, 0])
        self.declare_parameter('cell_size', [0.0, 0.0])
        self.declare_parameter('pile_size', 0.0)
        
        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('start_pose_sigma', [0.0, 0.0, 0.0])                
        
        self.declare_parameter('n_particles', 1)
        self.declare_parameter('wall_th', 0.5)
        self.declare_parameter('alpha', [.0, .0, .0, .0, .0, .0])
        self.declare_parameter('range_sigma', .1)

        self.get_parameter('n_particles').value
        
        self.declare_parameter('on_wall_th', 10)
        self.declare_parameter('wall_clear_th', 2)

        self.declare_parameter('n_updates', 2)
        
        on_wall_th = self.get_parameter('cell_size').value[0] / self.get_parameter('on_wall_th').value
        wall_clear_th = self.get_parameter('cell_size').value[0] / self.get_parameter('wall_clear_th').value
        
        self.SLAM = SLAM(self.get_parameter('lab_size').value,
                         self.get_parameter('cell_size').value,
                         self.get_parameter('pile_size').value,
                         self.get_parameter('start_pose').value,
                         self.get_parameter('start_pose_sigma').value,
                         self.get_parameter('n_particles').value,
                         self.get_parameter('wall_th').value,
                         self.get_parameter('alpha').value,
                         self.get_parameter('range_sigma').value,
                         on_wall_th,
                         wall_clear_th,
                         )
        self.print_func = self.get_logger().info

        # ROS2
        self.particles_pub = self.create_publisher(PoseArray, '~/particles', 10)
        self.wall_markers_pub = self.create_publisher(MarkerArray, '~/marker_map', 10)
        self.odom_pub = self.create_publisher(Odometry, '~/odom', 10)
        self.weights_marker_pub = self.create_publisher(Marker, '~/particle_weights', 1)

        self.service_sensors_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        self.cmd_cli = self.create_client(DirectionCommand, 'cmd')
        #self.cmd_cli.configure_introspection(self.get_clock(), qos_profile_system_default,
        #                                     ServiceIntrospectionState.CONTENTS)

        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.sens_cli = self.create_client(GetSensors, 'get_sensors')
        while not self.sens_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.cmd_srv = self.create_service(StepSLAMCommand, '~/cmd', self.step_cmd_cb, callback_group=self.callback_group)
        self.reset_srv = self.create_service(Trigger, '~/reset', self.reset_cb)
        
    def reset_cb(self, request, response):
        self.SLAM.init_pf(self.get_parameter('start_pose').value, self.get_parameter('start_pose_sigma').value)
        response.success = True
        response.message = f"SLAM is reseted to {self.get_parameter('start_pose').value} with {self.get_parameter('start_pose_sigma').value}"
        return response
        
    def init(self):
        #self.create_rate(5).sleep()
        sensors_request = GetSensors.Request()
        future = self.sens_cli.call_async(sensors_request)
        rclpy.spin_until_future_complete(self, future)

        sensors_response = future.result()
        if sensors_response.success:
            sensors_data = [[a, r] for r, a in zip(sensors_response.rangefinders_distances, sensors_response.rangefinders_angles)]
            
            self.SLAM.sensors_update(sensors_data, [0.3, 0.3, 1, 1, 0.3, 0.3])
            for _ in range(self.get_parameter('n_updates').value):                            
                self.SLAM.resampling()
            
            self.SLAM.update_map(sensors_data)
            self.get_logger().info(f"Init done")

            header = Header()
            header.frame_id = 'map'
            header.stamp = self.get_clock().now().to_msg()
            self.particles_pub.publish(self.SLAM.to_pose_array_msg(header))
            self.wall_markers_pub.publish(self.SLAM.to_marker_array_msg(header))
            self.odom_pub.publish(self.SLAM.to_odom_msg(header))

        else:
            self.get_logger().error(f"Sensors service result error")
        rclpy.spin_once(self)


        
    def step_cmd_cb(self, request, response):
        
        # only motion update if value != 0
        self.get_logger().info(f"Get cmd {request.direction} with {request.value}")
        sensor_w = None
        if request.value != 0.0:
            cmd_request = DirectionCommand.Request()
            cmd_request.direction = request.direction
            cmd_request.value = request.value

            cmd_event=Event()
            def done_callback(future):
                nonlocal cmd_event
                cmd_event.set()
            future_cmd = self.cmd_cli.call_async(cmd_request)
            future_cmd.add_done_callback(done_callback)

            dr, dyaw = 0, 0
            if request.direction == request.FORWARD:
                dr = request.value
                sensor_w = [1.0, 0.3, 0.3, 1.0, 0.3, 0.3]
            elif request.direction == request.BACKWARD:
                dr = -request.value
                sensor_w = [1.0, 0.3, 0.3, 1.0, 0.3, 0.3]
            elif request.direction == request.LEFT:
                dyaw = request.value
            elif request.direction == request.RIGHT:
                dyaw = -request.value

            self.SLAM.motion_update_step(dr, dyaw)

            cmd_event.wait()

        for _ in range(self.get_parameter('n_updates').value):
            sensors_request = GetSensors.Request()

            sens_event=Event()
            def done_callback(future):
                nonlocal sens_event
                sens_event.set()

            future = self.sens_cli.call_async(sensors_request)
            future.add_done_callback(done_callback)
            sens_event.wait()
            sensors_response = future.result()

            if not sensors_response.success:
                self.get_logger().error(f"Sensors service result error")
                return response

            sensors_data = [[a, r] for r, a in zip(sensors_response.rangefinders_distances, sensors_response.rangefinders_angles)]

            self.SLAM.sensors_update(sensors_data, sensor_w)
            self.SLAM.resampling()

            self.SLAM.update_pose_mean()
            #self.SLAM.update_pose_max()
            self.SLAM.update_map(sensors_data)

        response.success = sensors_response.success
        response.pose.pose = self.SLAM.to_pose_msg()
        response.h_walls = skukozjit_walls(self.SLAM.h_walls)
        response.v_walls = skukozjit_walls(self.SLAM.v_walls)

        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()

        self.odom_pub.publish(self.SLAM.to_odom_msg(header))
        #self.get_logger().info("Sent odom")
        self.particles_pub.publish(self.SLAM.to_pose_array_msg(header))
        #self.get_logger().info("Sent particles")
        self.weights_marker_pub.publish(self.SLAM.to_weights_marker_msg(header))
        #self.get_logger().info("Sent particle weights")
        self.wall_markers_pub.publish(self.SLAM.to_marker_array_msg(header))
        #self.get_logger().info("Sent map")
        
        return response


def main(args = None):
    rclpy.init(args = args)

    step_slam = StepSLAM()
    step_slam.init()
    executor = MultiThreadedExecutor()
    rclpy.spin(step_slam, executor)

    step_slam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
