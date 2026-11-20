from setuptools import find_packages, setup
from glob import glob
setup(name='omx_leader_approach', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/omx_leader_approach']),
                  ('share/omx_leader_approach', ['package.xml', 'README.md']),
                  ('share/omx_leader_approach/launch', glob('launch/*.launch.py')),
                  ('share/omx_leader_approach/config', glob('config/*.yaml'))],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['approach_node = omx_leader_approach.approach_node:main']})
