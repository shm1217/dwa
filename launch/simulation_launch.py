from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription()
    controller_node = Node(
        package= 'ros2_dwa_project',
        namespace= '',
        executable= 'cmd_publisher_node',
        output='screen'
    )
    ld.add_action(controller_node)

    return ld
