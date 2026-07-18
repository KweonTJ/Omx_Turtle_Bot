from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'turtlebot3_position'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'pyserial', 'PyYAML'],
    extras_require={
        'test': [
            'pytest',
            'PyYAML',
        ],
    },
    zip_safe=True,
    maintainer='kjy',
    maintainer_email='kjy@example.com',
    description=(
        'UWB global positioning and stop-and-go navigation for TurtleBot3.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'uwb_serial_node = turtlebot3_position.uwb_serial_node:main',
            'position_controller_node = turtlebot3_position.position_controller_node:main',
            'goal_console = turtlebot3_position.goal_console:main',
        ],
    },
)
