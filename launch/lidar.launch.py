from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_scan2',
            executable='sick_generic_caller',
            name='left_lidar',
            output='log',
            parameters=[
                {'scanner_name': 'sick_tim_5xx'},
                {'hostname': '172.16.127.11'},
                {'port': 2112},
                {'min_ang': 0.0},
                {'max_ang': 3.141},
                {'max_range': 10.0},
                {'frame_id': 'left_lidar'},
            ],
            remappings=[
                ('scan', '/left_lidar'),
                ('cloud', '/left_cloud'),
                ('imu', '/left_imu'),
            ],
        ),
        Node(
            package='sick_scan2',
            executable='sick_generic_caller',
            name='right_lidar',
            output='log',
            parameters=[
                {'scanner_name': 'sick_tim_5xx'},
                {'hostname': '172.16.127.10'},
                {'port': 2112},
                {'min_ang': 0.0},
                {'max_ang': 3.141},
                {'max_range': 10.0},
                {'frame_id': 'right_lidar'},
            ],
            remappings=[
                ('scan', '/right_lidar'),
                ('cloud', '/right_cloud'),
                ('imu', '/right_imu'),
            ],
        ),
        Node(
            package='ros_whill',
            executable='scan_fusion',
            name='scan_fusion',
            output='screen',
            parameters=[
                {'target_frame': 'base_link'},
                {'fused_cloud_topic': 'fused_cloud'},
                {'input_type': 'LaserScan'},
                {'scan_topics': ['/left_lidar', '/right_lidar']}
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='fused_cloud_to_fused_scan',
            remappings=[
                ('cloud_in', '/fused_cloud')
            ],
            parameters=[
                {'angle_min': -3.14159265},
                {'angle_max': 3.14159265},
                {'range_min': 0.0},
                {'range_max': 100.0}
            ]
        )
    ])
