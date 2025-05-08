from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    obs_time = LaunchConfiguration('obsTime_sec')
    ld = LaunchDescription()
    controller_node = Node(
        package= 'ros2_dwa_burger',
        namespace= '',
        executable= 'cmd_publisher_node',
        output='screen',
        parameters=[{'obsTime_sec': obs_time}]
    )

    ld.add_action(controller_node)

    return ld