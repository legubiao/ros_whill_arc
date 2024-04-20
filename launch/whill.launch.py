from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Robot Description
    xacro_file_name = 'modelc_with_lidar.xacro'
    xacro = os.path.join(
        get_package_share_directory('ros_whill'),
        'xacro',
        xacro_file_name)
    with open(xacro, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # Whill speed profile
    whill_speed_config = os.path.join(
        get_package_share_directory('ros_whill'),
        'params',
        'initial_speedprofile.yaml'
        )
    print(whill_speed_config)
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[rsp_params]
        ),
        Node(
            package='ros_whill',
            executable='ros2_whill',
            name='ros2_whill', 
            output='screen',
            namespace='whill',
            parameters=[
                whill_speed_config,
                {'send_interval': 10}
            ]
        )
    ])