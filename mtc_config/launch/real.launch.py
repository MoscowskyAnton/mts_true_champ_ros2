from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'mtc_config'

    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'real.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='mtc_drivers',
            namespace='mtc',
            executable='real_driver_node',
            name='driver',
            parameters=[config]            
        ),
        #Node(
            #package="rviz2",
            #namespace='',
            #executable="rviz2",
            #name="sim_rviz",
            #arguments=['-d', [rviz_config]]

        #),
        Node(
            package="mtc_slam",
            namespace="mtc",
            executable="step_slam_node",
            name="slam",
            parameters = [config],
            remappings=[('get_sensors', '/mtc/driver/get_sensors'),
                        ('cmd', '/mtc/dir_command')]
        )
    ])
