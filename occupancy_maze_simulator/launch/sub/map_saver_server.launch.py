#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import TimerAction


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server']  # map_serverを追加
    use_sim_time = True
    autostart = True
    
    # チルダPath指定に使えません！
    saved_map_path = '/home/izumita/.ros/save/selected_gridmap.yaml'
    start_map_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'yaml_filename': saved_map_path},
                        {'topic_name': "selected_gridmap"},
                        {'frame_id': "odom"}])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()

    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    #     TimerAction(
    #         period=0.0,
    #         actions=[start_lifecycle_manager_cmd],
    #     )
    # )

    return ld