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

    feedback_control_of_siso_system_node = Node(
        package=package_name,
        executable='feedback_control_of_siso_system',
        output="screen",
    )

    optimal_control_using_lqr_node = Node(
        package=package_name,
        executable='optimal_control_using_lqr',
        output="screen",
    )

    oinverted_pendulum_control_using_lqr_node = Node(
        package=package_name,
        executable='inverted_pendulum_control_using_lqr',
        output="screen",
    )

    nodes = [
        oinverted_pendulum_control_using_lqr_node,
    ]

    return LaunchDescription(nodes)
