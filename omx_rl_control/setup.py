from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'omx_rl_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        *[
            (
                os.path.join('share', package_name, os.path.dirname(path)),
                [path],
            )
            for path in glob('models/**/*', recursive=True)
            if os.path.isfile(path)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ktj',
    maintainer_email='kweontj0701@naver.com',
    description='Arm-only residual PPO runtime for OpenMANIPULATOR-X.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_control_node = omx_rl_control.rl_control_node:main',
        ],
    },
)
