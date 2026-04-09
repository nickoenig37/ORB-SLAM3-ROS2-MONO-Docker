import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_namespace = ""

    orb_slam3_launch_file_dir = os.path.join(
        get_package_share_directory('orb_slam3_ros2_wrapper'), 'launch')

    rover_rgbd_imu_launch_file_path = os.path.join(
        orb_slam3_launch_file_dir, 'rover_rgbd_imu.launch.py')

    rover_ros_params_path = os.path.join(
        get_package_share_directory('orb_slam3_ros2_wrapper'),
        'params', 'ros_params', 'rover-maxwell-rgbd-imu-ros-params.yaml')

    rover_rgbd_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_rgbd_imu_launch_file_path),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "params_file": rover_ros_params_path,
        }.items(),
    )

    monitor_enabled_arg = DeclareLaunchArgument(
        "monitor_enabled",
        default_value="true",
        description="Enable lightweight CPU/RAM monitor for the selected sensor configuration.",
    )

    # Print to screen: monitor should log to stdout
    monitor_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("monitor_enabled")),
        cmd=[
            "/root/colcon_ws/src/orb_slam3_ros2_wrapper/scripts/monitor_cpu_ram.sh",
            "--name", "rover_rgbd_imu",
            "--hz", "0.3",
            # no --out: stdout goes to screen via `output="screen"`
        ],
        output="screen",
        shell=False,
    )

    return LaunchDescription([
        monitor_enabled_arg,
        monitor_process,
        rover_rgbd_imu_launch,
    ])
