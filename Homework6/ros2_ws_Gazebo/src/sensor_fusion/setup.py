from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 加入 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='andrew',
    maintainer_email='2149366712@qq.com',
    description='传感器融合节点 - GPS/IMU/磁力计/里程计 卡尔曼滤波融合',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fusion_node = sensor_fusion.sensor_fusion_node:main',
        ],
    },
)
