import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    bringup_rmcs_flight = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
        		get_package_share_directory('rmcs_flight_bringup'),
        		'launch/launch.py'
        	)
        )
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
        		'launch/msg_MID360_launch.py'
        	)
        )
    )
    ld.add_action(bringup_mid360_driver)
    
    return ld