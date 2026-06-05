from setuptools import setup
import os
from glob import glob

package_name = 'turtle_figure8'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 package to control turtlesim in a figure-8 pattern',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'figure8_node = turtle_figure8.figure8_node:main',
        ],
    },
)