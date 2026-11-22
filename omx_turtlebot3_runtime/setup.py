from setuptools import setup, find_packages
from glob import glob
import os

assets = []
for directory, _, files in os.walk('assets'):
    if files:
        assets.append(('share/omx_turtlebot3_runtime/'+directory,
                       [directory+'/'+name for name in files]))
setup(name='omx_turtlebot3_runtime', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/omx_turtlebot3_runtime']),
                  ('share/omx_turtlebot3_runtime', ['package.xml', 'README.md']),
                  ('share/omx_turtlebot3_runtime/config', glob('config/*.yaml')),
                  ('share/omx_turtlebot3_runtime/launch', glob('launch/*.launch.py'))]+assets,
      install_requires=['setuptools'], zip_safe=False,
      entry_points={'console_scripts': ['runtime = omx_turtlebot3_runtime.node:main',
                                       'runtime_cpp = omx_turtlebot3_runtime.node:main_cpp',
                                       'preflight = omx_turtlebot3_runtime.preflight:main']})
