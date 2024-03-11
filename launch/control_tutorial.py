from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'arcanain_modern_control_tutorial'

    unforced_response_with_mass_spring_damper_node = Node(
        package=package_name,
        executable='unforced_response_with_mass_spring_damper',
        output="screen",
    )

    nodes = [
        unforced_response_with_mass_spring_damper_node,
    ]

    return LaunchDescription(nodes)
