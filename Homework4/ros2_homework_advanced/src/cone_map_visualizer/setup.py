import os
from setuptools import find_packages, setup

package_name = 'cone_map_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/cone_map.rviz']),
        (os.path.join('share', package_name, 'launch'), ['launch/visualize_cone_map.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='2149366712@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cone_map_visualizer_node = cone_map_visualizer.cone_map_visualizer_node:main'
        ],
    },
)
