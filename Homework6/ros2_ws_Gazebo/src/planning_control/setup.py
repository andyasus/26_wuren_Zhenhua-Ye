from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'planning_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='student',
    maintainer_email='user@example.com',
    description='路径规划与车辆控制',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = planning_control.path_planner_node:main',
            'controller_node = planning_control.controller_node:main',
        ],
    },
)
