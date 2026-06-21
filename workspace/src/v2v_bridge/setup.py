import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'v2v_bridge'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='artemkondratev5@gmail.com',
    description='UDP telemetry bridge between robots with isolated DDS domains',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v2v_sender = v2v_bridge.v2v_sender:main',
            'v2v_receiver = v2v_bridge.v2v_receiver:main',
        ],
    },
)
