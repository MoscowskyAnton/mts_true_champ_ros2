import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

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



class PIDControl(Node):
    # node class
    def __init__(self):
        super().__init__('pid_control')

        # read parameters
        self.declare_parameter('P_mov', 1.)
        self.declare_parameter('I_mov', 0.)
        self.declare_parameter('D_mov', 0.)
        self.declare_parameter('P_rot', 1.)
        self.declare_parameter('I_rot', 0.)
        self.declare_parameter('D_rot', 0.)
        self.declare_parameter('max_movement_vel', 0.1)  # m/s
        self.declare_parameter('max_rotation_vel', 1.)  # rad/s
        self.declare_parameter('buffer_len', 3)  # rad/s
        self.declare_parameter('timer_period_s', 0.1)  # s

        self.declare_parameter('delay_before_stop_s', 1.)

        # time of last command
        self.last_command_time = None
        # create state variables
        self.PID_mov = PID(self, 'P_mov', 'I_mov', 'D_mov', 'max_movement_vel', 'buffer_len')
        self.PID_rot = PID(self, 'P_rot', 'I_rot', 'D_rot', 'max_rotation_vel', 'buffer_len')
        self.cmd_mov = 0.
        self.cmd_rot = 0.

        # create ROS infrastructure
        self.cmd_out_pub = self.create_publisher(TwistStamped, 'cmd_vel_pid', 1)
        self.cmd_in_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cb_cmd_vel, 1)
        # TODO add state when it will be published
        self.cmd_timer = self.create_timer(self.get_parameter('timer_period_s').value, self.cb_timer)

    def cb_cmd_vel(self, msg):
        # read input values
        self.cmd_mov = msg.twist.linear.x
        self.cmd_rot = msg.twist.angular.z
        self.last_command_time = self.get_clock().now()

    def cb_timer(self):
        # publish command for the output;
        # TODO change when state will be available
        # check if commands are absent too much time
        if self.last_command_time is None:
            return
        dur = rclpy.duration.Duration(seconds=self.get_parameter('delay_before_stop_s').value)
        if self.get_clock().now() - self.last_command_time > dur:
            return
        cmd_mov_out = self.PID_mov.get_signal(self.cmd_mov)
        cmd_rot_out = self.PID_rot.get_signal(self.cmd_rot)
        # publish command
        out_msg = TwistStamped()
        out_msg.twist.linear.x = cmd_mov_out
        out_msg.twist.angular.z = cmd_rot_out
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_out_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    pid_control = PIDControl()

    rclpy.spin(pid_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
