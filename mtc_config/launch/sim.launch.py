from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'mtc_config'

    rviz_config = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'sim.rviz'
        )

    #pkg_dir = os.popen('source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd' % pkg_name).read().strip()
    #rviz_config = os.path.join(pkg_dir, 'rviz','sim.rviz')
    print(rviz_config)


    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'sim.yaml'
        )
    #print(slam_config)

    return LaunchDescription([
         # Node(
         #     package='joy',
         #     namespace='mtc',
         #     executable='joy_node',
         #     name='joy'
         # ),
         # Node(
         #     package='teleop_twist_joy',
         #     namespace='mtc',
         #     executable='teleop_node',
         #     name='teleop_node',
         #     parameters=[
         #         {'publish_stamped_twist': True},
         #         {'enable_button': 1},
         #         {"axis_linear.x": 1},
         #         {"axis_angular.z": 0},
         #         {"scale_linear": {"x": 200.}},
         #         {"scale_angular": {"yaw":100.}}]
         # ),
        Node(
            package='mtc_drivers',
            namespace='mtc',
            executable='sim_driver_node',
            name='driver',
            parameters=[config]
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        ),
        Node(
            package="rviz2",
            namespace='',
            executable="rviz2",
            name="sim_rviz",
            arguments=['-d', [rviz_config]]
        ),
        Node(
            package="mtc_slam",
            namespace="mtc",
            executable="step_slam_node",
            name="slam",
            parameters = [config],
            remappings=[('get_sensors', '/mtc/driver/get_sensors'),
                        ('cmd', '/mtc/dir_command')]
        ),
        Node(
            package="mtc_dir_commands",
            namespace='mtc',
            executable="mtc_dir_commands_node",
            name="dir_commands",
            parameters=[config],
            remappings=[('true_odom', 'driver/true_odom')]
        ),
        Node(
            package='mtc_controllers',
            namespace='mtc',
            executable='astar_step_controller_node',
            name='astar_controller',
            parameters=[config],
            remappings=[]
        )
        # Node(
        #     package='carrot_trajectory_following',
        #     namespace='mtc',
        #     executable='carrot_standalone',
        #     name='carrot_standalone',
        #     parameters=[config],
        #     remappings=[('step_slam_command', 'slam/cmd')],
        # ),
        # Node(
        #     package='carrot_trajectory_following',
        #     namespace='mtc',
        #     executable='rviz2path_node',
        #     #prefix=['konsole -e python3 -m pdb'],
        #     name='rviz2path',
        # )
    ])
