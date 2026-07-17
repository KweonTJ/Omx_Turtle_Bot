from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'omx_eef_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ktj',
    maintainer_email='kweontj0701@naver.com',
    description='ArUco-only object pose estimation for an EEF camera.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'eef_vision_node = omx_eef_vision.eef_vision_node:main',
        ],
    },
)
