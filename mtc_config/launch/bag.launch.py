from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'mtc_config'

    rviz_config = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'bag.rviz'
        )

    #pkg_dir = os.popen('source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd' % pkg_name).read().strip()
    #rviz_config = os.path.join(pkg_dir, 'rviz','sim.rviz')
    print(rviz_config)


    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'real.yaml'
        )
    #print(slam_config)

    return LaunchDescription([
        Node(
            package="rviz2",
            namespace='',
            executable="rviz2",
            name="sim_rviz",
            arguments=['-d', [rviz_config]]

        ),
        Node(
            package="mtc_drivers",
            namespace='mtc',
            executable="bag_vis_node",
            name="bag_vis_node",
            remappings=[('slam/odom', ('slam_sim/odom'))]
        ),
        Node(
            package="mtc_slam",
            namespace='mtc',
            executable="bag_play_node",
            name="bag_play"
            #remappings=[('slam/cmd')]
        ),
        Node(
            package="mtc_slam",
            namespace='mtc',
            executable="bag_plug_node",
            name="bag_plug",
            parameters = [{"wait_time": 3.0}]
            #remappings=[('slam/cmd')]
        ),
        Node(
            package="mtc_slam",
            namespace="mtc",
            executable="step_slam_node",
            name="slam",
            parameters = [config],
            remappings=[('get_sensors', '/mtc/bag_play/get_sensors'),
                        ('cmd', '/mtc/dir_command'),
                        ('~/particles', 'slam_sim/particles'),
                        ('~/odom', 'slam_sim/odom'),
                        ('~/particle_weights', 'slam_sim/particle_weights'),
                        ('~/marker_map', 'slam_sim/marker_map'),
                        ('~/proc_cmd', 'slam_sim/proc_cmd')]
        )

    ])
