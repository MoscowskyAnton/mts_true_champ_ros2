import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from mtc_msgs.msg import SetPWM

class Twist2PWM(Node):
    def __init__(self):
        super().__init__('twist2pwm')

        # parameters
        self.max_allowed_pwm = 255
        # make real limit lesser
        self.declare_parameter('max_pwm', 100)
        self.declare_parameter('cmd_time_s', 0.1)
        self.declare_parameter('wheel_radius_m', 0.01)
        self.declare_parameter('dist_between_wheels_m', 0.1)
        # coefficient to calculate pwms from radians
        self.declare_parameter('rads_to_pwm_coeff', 100)

        # ROS infrastructure
        self.sub_twist = self.create_subscription(TwistStamped, 'cmd_vel_pid', self.cb_cmd_vel, 1)
        self.pub_pwm = self.create_publisher(SetPWM, 'pwm', 1)

    def cb_cmd_vel(self, msg):
        # calculate wheel commands from input
        # TODO calculate signals better
        vel = msg.twist.linear.x
        angle_rad = vel / (2 * math.pi * self.get_parameter('wheel_radius_m').value)
        pwm_mov = angle_rad * self.get_parameter('rads_to_pwm_coeff').value
        # A. If rotation is zero, commands should be equal
        if abs(msg.twist.angular.z) < 1e-6:
            pwm_rot = 0.
        else:
            # B. Rotation is not zero; wheel distances differs
            wvel = msg.twist.angular.z
            angle_rot = wvel * self.get_parameter('dist_between_wheels_m').value / (2. * self.get_parameter('wheel_radius_m').value)
            pwm_rot = angle_rot * self.get_parameter('rads_to_pwm_coeff').value
        # output signal
        signal_l = int(pwm_mov - pwm_rot)
        signal_r = int(pwm_mov + pwm_rot)
        max_pwm = self.get_parameter('max_pwm').value
        # trim signals
        if signal_l > max_pwm:
            signal_l = max_pwm
        if signal_l < -max_pwm:
            signal_l = -max_pwm
        if signal_r > max_pwm:
            signal_r = max_pwm
        if signal_r < -max_pwm:
            signal_r = -max_pwm
        # prepare and publish result
        pwm = SetPWM()
        pwm.left_pwm = signal_l
        pwm.right_pwm = signal_r
        pwm.time = self.get_parameter('cmd_time_s').value
        self.pub_pwm.publish(pwm)

def main(args=None):
    #print('Hi from mtc_drivers.')
    rclpy.init(args=args)

    twist2pwm = Twist2PWM()

    rclpy.spin(twist2pwm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist2pwm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
