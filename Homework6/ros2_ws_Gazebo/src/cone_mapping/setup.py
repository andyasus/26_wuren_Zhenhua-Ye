from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cone_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='user',
    maintainer_email='user@test.com',
    description='锥桶检测 + 地标匹配定位',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cone_detector = cone_mapping.cone_detector:main',
            'landmark_localizer = cone_mapping.landmark_localizer:main',
        ],
    },
)
