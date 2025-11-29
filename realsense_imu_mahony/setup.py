from setuptools import setup
import os
from glob import glob

package_name = 'realsense_imu_mahony'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들을 share/패키지명/launch 폴더로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Mahony IMU filter node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 노드 실행 명령어 등록
            'imu_mahony_node = realsense_imu_mahony.imu_mahony_node:main',
        ],
    },
)