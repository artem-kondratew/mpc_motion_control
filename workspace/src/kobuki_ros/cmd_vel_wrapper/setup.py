from setuptools import find_packages, setup

package_name = 'cmd_vel_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='artemkondratev5@gmail.com',
    description='remapping cmd_vel msgs from /cmd_vel to /commands/velocity',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_wrapper = cmd_vel_wrapper.main:main',
        ],
    },
)
