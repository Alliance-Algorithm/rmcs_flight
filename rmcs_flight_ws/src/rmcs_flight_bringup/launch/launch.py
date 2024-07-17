from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("rmcs_flight_bringup"), "config", "flight_controller.yaml"])
    ]

    flight_controller_node = Node(
        package="flight_controller",
        executable="flight_controller_exe",
        parameters=params,
        output="screen"
    )
    
    ld = LaunchDescription()

    ld.add_action(flight_controller_node)

    return ld