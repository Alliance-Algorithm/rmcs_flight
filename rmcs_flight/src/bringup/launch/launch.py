import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore


def generate_launch_description():
    ld = LaunchDescription()

    params = [
        PathJoinSubstitution([FindPackageShare("bringup"), "config", "flight_controller.yaml"])
    ]

    bringup_rmcs_flight = Node(
        package="flight_controller",
        executable="flight_controller_exe",
        parameters=params,
        output="screen",
        respawn=True,
        respawn_delay=1
    )
    ld.add_action(bringup_rmcs_flight)

    bringup_rmcs_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
        		get_package_share_directory('rmcs_slam'),
        		'launch/slam.py'
        	)
        )
    )
    ld.add_action(bringup_rmcs_slam)

    bringup_mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
        		get_package_share_directory('livox_ros_driver2'),
        		'launch/mid360.py'
        	)
        )
    )
    ld.add_action(bringup_mid360_driver)
    
    return ld