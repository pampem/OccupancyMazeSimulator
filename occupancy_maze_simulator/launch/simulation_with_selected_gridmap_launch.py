# Author: Masashi Izumita
import os
import csv
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def save_params_to_csv(file_path, params):
    """
    Save parameters to a CSV file.
    Expects a dictionary of parameters where keys are parameter names and values are parameter values.
    """
    with open(file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        for param_name, param_value in params.items():
            writer.writerow([param_name, param_value])


def generate_launch_description():
    ld = LaunchDescription()

    package_share_directory = get_package_share_directory(
        'occupancy_maze_simulator')

    # Generate CSV filename with current date and time
    current_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_csv_file_path = os.path.join(f'data_rosparam_{current_date}.csv')

    namespace = 'drone1'

    omc_params = {
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
        "robot_pose_topic": "mavros/vision_pose/pose",
        "robot_velocity_topic": "mavros/setpoint_velocity/cmd_vel_unstamped",
    }

    omc_node = Node(
        package="occupancy_maze_simulator",
        executable="occupancy_maze_simulator_node",
        output="screen",
        parameters=[omc_params],
        arguments=["--ros-args", "--log-level", "info"],
        namespace=namespace,
    )

    path_planner_params = {
        "attractive_force.max_distance": 2.0,
        "attractive_force.gain": 0.5,
        "repulsive_force.influence_distance": 2.0,
        "repulsive_force.gain": 2.0,
        "repulsive_force.max": 3.0,
        "rotational_force.gain": 1.0,
        "rotational_force.influence_distance": 3.0,
        "rotational_force.max": 2.0,
        "robot_velocity.max": 2.0,
        "robot_pose_topic": "mavros/vision_pose/pose",
        "gridmap_topic": "slam_gridmap",
        "robot_velocity_topic": "mavros/setpoint_velocity/cmd_vel_unstamped",
    }

    path_planner_node = Node(
        package="path_planner",
        executable="path_planner_node",
        output="screen",
        parameters=[path_planner_params],
        arguments=["--ros-args", "--log-level", "info"],
        namespace=namespace,
    )

    rviz_config_path = os.path.join(
        package_share_directory, 'rviz', 'config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path],
        # TODO: 複数台シミュレーションを実装した場合、ここはnamespaceを適用しないなど対応
        namespace=namespace,
    )

    # Save applied parameters to a CSV file for record-keeping
    applied_params = {**omc_params, **path_planner_params}
    save_params_to_csv(save_csv_file_path, applied_params)

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
