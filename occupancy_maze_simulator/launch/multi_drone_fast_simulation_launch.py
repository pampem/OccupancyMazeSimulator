# Author: Masashi Izumita
# 4台同時評価を行うためのLaunchスクリプト
# 4つのpath_plannerが一つの迷路を共有する。
# quadraple_fast_simulation_launch.pyとの主な違いは、
# omcが一つしか立ち上がらないこと。
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
    with open(file_path, 'w', newline='') as csv_file, open(file_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        for param_name, param_value in params.items():
            writer.writerow([param_name, param_value])


def generate_launch_description():
    ld = LaunchDescription()

    occupancy_maze_simulator_dir = get_package_share_directory(
        'occupancy_maze_simulator')

    # Generate CSV filename with current date and time
    current_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    evaluation_dir = os.path.join(os.path.expanduser("~"), ".ros", "occupancy_maze_simulator", "evaluation")
    os.makedirs(evaluation_dir, exist_ok=True)
    save_csv_file_path = os.path.join(evaluation_dir, f"{current_date}_data_rosparam.csv")

    namespaces = []
    for i in range(4):
        namespaces.append(f"drone{i}")

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
        "max_trial_count": 25, # 1/4でよい！
        "simulation_timeout": 100.0,
        "robot_pose_topic": "mavros/vision_pose/pose",
        "robot_velocity_topic": "mavros/setpoint_velocity/cmd_vel_unstamped",
        "odom_frame": "",  # will be set below
        "base_link_frame": "",  # will be set below
        "csv_stat_file_name_prefix": "",  # will be set below
    }

    path_planner_params = {
        "attractive_force.max_distance": 2.0,
        "attractive_force.gain": 1.0,
        "repulsive_force.influence_distance": 10.0,
        "repulsive_force.max": 2.0,
        "repulsive_force.gain": 2.0,
        "rotational_force.influence_distance": 4.0,
        "rotational_force.max": 2.0,
        "rotational_force.gain": 2.0,
        "robot_velocity.max": 3.0,
        "robot_pose_topic": "mavros/vision_pose/pose",
        "gridmap_topic": "slam_gridmap",
        "robot_velocity_topic": "mavros/setpoint_velocity/cmd_vel_unstamped",
        "mode": "fast",
        "lidar_points_gridmap.resolution": 0.5,
        "lidar_points_gridmap.width": 20,
        "lidar_points_gridmap.height": 20,
        "lidar_points_gridmap.min_points_to_fill_cell": 2,
        "attractive_gain_with_rotational_force": 0.5,
        "odom_frame_id": "",
        "base_link_frame_id": "",
    }

    for ns in namespaces:
        updated_omc_params = dict(omc_params)
        updated_omc_params["odom_frame_id"] = f"{ns}_odom"
        updated_omc_params["base_link_frame_id"] = f"{ns}_base_link"
        updated_omc_params["csv_stat_file_name_prefix"] = f"{ns}_"

        updated_path_planner_params = dict(path_planner_params)
        updated_path_planner_params["odom_frame_id"] = f"{ns}_odom"
        updated_path_planner_params["base_link_frame_id"] = f"{ns}_base_link"

        omc_node = Node(
            package="occupancy_maze_simulator",
            executable="occupancy_maze_simulator_node",
            output="screen",
            parameters=[updated_omc_params],
            arguments=["--ros-args", "--log-level", "info"],
            namespace=ns,
        )

        path_planner_node = Node(
            package="path_planner",
            executable="path_planner_node",
            output="screen",
            parameters=[updated_path_planner_params],
            arguments=["--ros-args", "--log-level", "info"],
            namespace=ns,
        )

        global_planner_node = Node(
            package="path_planner",
            executable="global_planner_node",
            output="screen",
            parameters=[
                {
                    "mode": "static",
                    "pose_topic": "mavros/vision_pose/pose",
                    "goal_reached_tolerance": 1.0,
                    "waypoints0.x": 20.0,
                    "waypoints0.y": 20.0,
                    "waypoints0.z": 0.0,
                    "odom_frame_id": f"{ns}_odom",
                }
            ],
            arguments=["--ros-args", "--log-level", "info"],
            namespace=ns,
        )

        rviz_config_path = os.path.join(
            occupancy_maze_simulator_dir, 'rviz', f'{ns}_config.rviz')

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path],
            namespace=ns,
        )

        ld.add_action(omc_node)
        ld.add_action(path_planner_node)
        ld.add_action(global_planner_node)
        ld.add_action(rviz)

    # Save applied parameters to a CSV file for record-keeping
    applied_params = {**omc_params, **path_planner_params}
    save_params_to_csv(save_csv_file_path, applied_params)

    ld.add_action(
        ExecuteProcess(
            cmd=["echo", "simulation setup complete."], output="screen")
    )

    return ld


if __name__ == "__main__":
    generate_launch_description()
