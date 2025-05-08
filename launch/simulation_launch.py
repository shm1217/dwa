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
    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        output="screen",
        name="octomap_server",
        parameters=[
            {"frame_id": "map",
             "resolution": 0.1}
        ],
        remappings=[
            ('cloud_in', '/scan_matched_points2'),
        ]
    )

    ld.add_action(controller_node)
    ld.add_action(octomap_node)

    return ld
