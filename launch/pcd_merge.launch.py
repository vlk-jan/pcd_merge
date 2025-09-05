from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pcd_merge",
                executable="pcd_merge",
                name="pcd_merge_node",
                output="screen",
                parameters=[
                    {
                        "num_clouds": 2,
                        "queue_size": 10,
                        "target_frame": "base_link",
                        "hz": 10.0,
                    }
                ],
                remappings=[
                    ("input_cloud_0", "livox/lidar_front"),
                    ("input_cloud_1", "livox/lidar_rear"),
                ],
            )
        ]
    )
