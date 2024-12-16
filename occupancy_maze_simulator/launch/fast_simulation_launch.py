# Author: Masashi Izumita
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    omc_node = Node(
        package="occupancy_maze_simulator",
        executable="occupancy_maze_simulator_node",
        output="screen",
        parameters=[
            {
                "gridmap.resolution": 1.0,
                "gridmap.x": 50.0,
                "gridmap.y": 50.0,
                "gridmap.origin_x": -25.0,
                "gridmap.origin_y": -25.0,
                "start_pose.x": -20.0,
                "start_pose.y": -20.0,
                "target_pose.x": 20.0,
                "target_pose.y": 20.0,
                "maze.density": 0.1,
                "max_trial_count": 100,
                "simulation_timeout": 100.0,
                "robot_pose_topic": "drone1/mavros/vision_pose/pose",
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    path_planner_node = Node(
        package="path_planner",
        executable="path_planner_node",
        output="screen",
        parameters=[
            {
                "attractive_force.max_distance": 2.0,
                "attractive_force.gain": 0.5,
                "repulsive_force.influence_distance": 2.0,
                "repulsive_force.gain": 2.0,
                "repulsive_force.max": 3.0,
                "rotational_force.gain": 1.0,
                "rotational_force.influence_distance": 3.0,
                "rotational_force.max": 2.0,
                "robot_pose_topic": "drone1/mavros/vision_pose/pose",
                "gridmap_topic": "slam_gridmap",
                "robot_velocity_topic": "drone1/mavros/setpoint_velocity/cmd_vel_unstamped",
            }
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # occupancy_maze_simulator/occupancy_maze_simulator/rviz/config.rviz
    # シンボリックリンクなので変更の保存も効く
    package_share_directory = get_package_share_directory('occupancy_maze_simulator')
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    ld.add_action(omc_node)
    ld.add_action(path_planner_node)
    ld.add_action(rviz)
    ld.add_action(
        ExecuteProcess(
            cmd=["echo", "real flight setup complete."], output="screen")
    )

    return ld


if __name__ == "__main__":
    generate_launch_description()
