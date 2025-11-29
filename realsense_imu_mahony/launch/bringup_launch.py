import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'realsense_imu_mahony'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 사용자가 만든 RealSense 런치 파일 포함
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rs_custom_launch.py')
        )
    )

    # 2. Mahony IMU 필터 노드 실행
    mahony_node = Node(
        package=pkg_name,
        executable='imu_mahony_node',
        name='imu_mahony',
        output='screen',
        parameters=[{
            'gyro_topic': '/camera/gyro/sample',
            'accel_topic': '/camera/accel/sample',
            'Kp': 2.0,
            'Ki': 0.0,
            'use_bias_calib': True
        }]
    )

    return LaunchDescription([
        rs_launch,
        mahony_node
    ])