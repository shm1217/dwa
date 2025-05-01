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
