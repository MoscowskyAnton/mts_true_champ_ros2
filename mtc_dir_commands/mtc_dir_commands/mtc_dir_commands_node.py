import math
import time
import threading

from mtc_msgs.srv import DirectionCommand

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

#from rclpy.qos import qos_profile_system_default
#from rclpy.service_introspection import ServiceIntrospectionState

from mtc_msgs.msg import SetPWM
from nav_msgs.msg import Odometry

def angle_sum(a1, a2):
    diff = a1 + a2
    return (diff + math.pi) % (2*math.pi) - math.pi

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

class PID:
    # PID controller to calculate control signals
    def __init__(self, param_source, P_name, I_name, D_name, max_signal_name, max_buffer_len_name):
        # param_source is a node containing values for all parameters
        self.param_source = param_source
        self.P_name = P_name
        self.I_name = I_name
        self.D_name = D_name
        self.max_buffer_len_name = max_buffer_len_name
        self.max_signal_name = max_signal_name

        self.reset()

    def reset(self):
        # drop all previous info to use the controller
        # for a new task
        self.buffer = []

    def get_signal(self, error):
        # read parameters
        P = self.param_source.get_parameter(self.P_name).value
        I = self.param_source.get_parameter(self.I_name).value
        D = self.param_source.get_parameter(self.D_name).value
        max_signal = self.param_source.get_parameter(self.max_signal_name).value
        max_buffer_len = self.param_source.get_parameter(self.max_buffer_len_name).value
        # calculate control signal
        # append signal to the buffer
        self.buffer.append(error)
        if len(self.buffer) > max_buffer_len:
            del self.buffer[0]
        # calculate components
        p_value = P * error
        i_value = I * sum(self.buffer)
        d_value = D * (self.buffer[-1] - self.buffer[-2]) if len(self.buffer) > 1 else 0.
        sum_cmd = p_value + i_value + d_value
        # process sum
        if sum_cmd > max_signal:
            sum_cmd = max_signal
        elif sum_cmd < -max_signal:
            sum_cmd = -max_signal
        return sum_cmd

class DirCommandService(Node):

    def __init__(self):
        super().__init__('dir_commands')

        # parameters:
        # rotation velocity
        self.declare_parameter('waiting_period_s', 0.1)
        # reserve coefficients: how many times should the service wait for the command ending;
        # waiting time = waiting_reserve_coeff * vel_mov * goal
        #self.declare_parameter('waiting_reserve_coeff_rot', 3.)
        #self.declare_parameter('waiting_reserve_coeff_mov', 3.)
        # PID parameters
        self.declare_parameter('P_mov', 1.)
        self.declare_parameter('I_mov', 0.)
        self.declare_parameter('D_mov', 0.)
        self.declare_parameter('P_rot', 1.)
        self.declare_parameter('I_rot', 0.)
        self.declare_parameter('D_rot', 0.)
        self.declare_parameter('max_movement_vel', 100)  # pwm
        self.declare_parameter('max_rotation_vel', 150)  # pwm
        self.declare_parameter('buffer_len', 3)


        # state variables:
        # current coordinates and orientation
        self.x = None
        self.y = None
        self.ori = None
        self.state_lock = threading.Lock()
        self.unified_callback_group = ReentrantCallbackGroup()
        self.loop_rate = self.create_rate(1. / self.get_parameter('waiting_period_s').value,
                                          self.get_clock())
        self.PID_mov = PID(self, 'P_mov', 'I_mov', 'D_mov', 'max_movement_vel', 'buffer_len')
        self.PID_rot = PID(self, 'P_rot', 'I_rot', 'D_rot', 'max_rotation_vel', 'buffer_len')

        # ROS infrastructure
        # publisher for commands (now it is TwistStamped only)
        self.pub_pwm = self.create_publisher(SetPWM, 'driver/pwm', 1)
        # subscriber for precise position
        self.cb_true_odom = self.create_subscription(Odometry, 'true_odom', self.cb_true_odom,
                                                     1, callback_group=self.unified_callback_group)
        # service
        self.srv = self.create_service(DirectionCommand, 'dir_command',
                                       self.cb_dir_command, callback_group=self.unified_callback_group)
        #self.srv.configure_introspection(self.get_clock(), qos_profile_system_default,
        #                                 ServiceIntrospectionState.CONTENTS)

    def get_command_from_PID(self, direction, error):
        if direction in [0,1]:
            signal = self.PID_mov.get_signal(error)
            return (signal, 0) if direction == 1 else (-signal, 0)
        else:
            return (0, self.PID_rot.get_signal(error))

    def cb_dir_command(self, request, response):
        # if current position is not known, return false
        if self.x is None:
            response.success = False
            return response
        # fix initial position
        init_x = self.x
        init_y = self.y
        init_ori = self.ori
        dir_x = math.cos(init_ori)
        dir_y = math.sin(init_ori)

        # mark start time
        start_time = self.get_clock().now()
        # estimate end time
        move_duration = rclpy.duration.Duration(seconds=10) # TODO improve time period
        end_time = start_time + move_duration

        num_goals_to_finish = 4
        dist_tolerance = 0.01
        angle_tolerance = 2. * math.pi / 180.
        curr_num_goals = 0

        # extra addition to decrease dead zone
        extra_impulse = 0
        iter_num = 1

        # control command execution
        while self.get_clock().now() < end_time:
            # get current position
            self.state_lock.acquire()
            curr_x = self.x
            curr_y = self.y
            curr_ori = self.ori
            self.state_lock.release()
            # check ifthe goal reached
            goal_reached = False
            error = 0
            if request.direction in [0,1]: #DirectionCommand.FORWARD,
                                     #DirectionCommand.BACKWARD]:
                # for direct motion: find projection of the
                # current position to the initial direction
                dx = curr_x - init_x
                dy = curr_y - init_y
                dist = dx * dir_x + dy * dir_y
                self.get_logger().error(f"movement; dist is {dist}; error is {dist-request.value}")
                if abs(dist - request.value) < dist_tolerance:
                    goal_reached = True
                    curr_num_goals += 1
                else:
                    if request.direction == 0:
                        error = dist - request.value
                    else:
                        error = -(dist + request.value)
                    curr_num_goals = 0
            elif request.direction in [2,3]: #DirectionCommand.LEFT,
                                       #DirectionCommand.RIGHT]:
                # for rotation: estimate angle difference
                # 1. Correct final orientation --- initial + request
                goal_ori = angle_sum(init_ori, request.value if request.direction == 2 else -request.value)
                # 2. Difference between current and goal orientations
                ori_diff = angle_diff(curr_ori, goal_ori)
                #self.get_logger().error(f"rotation; diff is {ori_diff}, curr is {curr_ori}, goal is {goal_ori}")
                if abs(ori_diff) < angle_tolerance:
                    goal_reached = True
                    curr_num_goals += 1
                else:
                    error = -ori_diff
                    curr_num_goals = 0
            # if movement should be stopped, send zero values.
            if goal_reached and curr_num_goals >= num_goals_to_finish:
                # send last zero command
                cmd.left_pwm = 0
                cmd.right_pwm = 0
                cmd.time = self.get_parameter('waiting_period_s').value
                self.pub_pwm.publish(cmd)
                self.get_logger().info(f"Movement finished successfully")
                response.success = True
                return response
            else:
                # calculate command
                if goal_reached:
                    cmd_vel, cmd_wvel = 0., 0.
                else:
                    cmd_vel, cmd_wvel = self.get_command_from_PID(request.direction, error)

                # make command
                cmd = SetPWM()
                cmd.left_pwm = int(cmd_vel - cmd_wvel)
                cmd.right_pwm = int(cmd_vel + cmd_wvel)
                cmd.time = self.get_parameter('waiting_period_s').value
                # add extra impulse in the beginning of movement
                extra = int(extra_impulse / math.sqrt(iter_num))
                #self.get_logger().error(f"command before extra is {cmd.left_pwm} {cmd.right_pwm}")
                if cmd.left_pwm > 0:
                    cmd.left_pwm += extra
                elif cmd.left_pwm <0:
                    cmd.left_pwm -= extra
                if cmd.right_pwm > 0:
                    cmd.right_pwm += extra
                elif cmd.right_pwm <0:
                    cmd.right_pwm -= extra
                # send command
                self.pub_pwm.publish(cmd)
                self.get_logger().info(f"Movement command is {cmd.left_pwm} {cmd.right_pwm}")

            # wait
            self.loop_rate.sleep()
            iter_num += 1

        # movement was non finished in time, return false
        cmd.left_pwm = 0
        cmd.right_pwm = 0
        cmd.time = self.get_parameter('waiting_period_s').value
        self.pub_pwm.publish(cmd)
        self.get_logger().error(f"Movement finished with error")

        response.success = False
        return response


    def cb_true_odom(self, msg):
        # read true robot position from the model
        self.state_lock.acquire()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.ori = 2 * math.atan2(quat.z, quat.w)
        self.state_lock.release()
        #self.get_logger().info(f'position: {self.x} {self.y} {self.ori}')

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    dir_command_service = DirCommandService()
    try:
        rclpy.spin(dir_command_service, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
