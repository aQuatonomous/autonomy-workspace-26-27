from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_server_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@example.com',
    description='Web-based 2D map visualization for boat position and global detections',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_visualizer_node = web_server_map.map_visualizer_node:main',
        ],
    },
)
