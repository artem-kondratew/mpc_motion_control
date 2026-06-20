from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autoware_integration_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='d.demidov@automacon.ru',
    description='Empty message publisher and twist estimator nodes',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'empty_publisher = autoware_integration_tools.empty_publisher:main',
            'debug_odometry_simulator = autoware_integration_tools.debug_odometry_simulator:main',
            'twist_estimator = autoware_integration_tools.twist_estimator:main',
            'foxglove_visualizer = autoware_integration_tools.foxglove_visualizer:main',
        ],
    },
)
