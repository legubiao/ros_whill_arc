from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_whill',  # 请替换为你的包名
            executable='ros2_whill',  # 请替换为你的可执行文件名
            name='ros2_whill_node',  # 节点的名字
            output='screen',
            parameters=[
                {'send_interval': 10}
            ]
        )
    ])