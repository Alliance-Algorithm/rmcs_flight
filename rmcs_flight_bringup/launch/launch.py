from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore



def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("rmcs_flight_bringup"), "config", "flight_controller.yaml"])
    ]

    flight_controller_node = Node(
        package="flight_controller",
        executable="flight_controller_exe",
        parameters=params,
        output="screen",
        respawn=True,
        respawn_delay=1
    )
    
    ld = LaunchDescription()

    ld.add_action(flight_controller_node)

    return ld