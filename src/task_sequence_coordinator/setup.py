from setuptools import find_packages, setup

package_name = 'task_sequence_coordinator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python-headless'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@example.com',
    description='Task sequence coordinator for timed task execution and sound-based navigation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_sequence_coordinator = task_sequence_coordinator.task_sequence_coordinator_node:main',
            'patrol_boat_detector = task_sequence_coordinator.patrol_boat_detector:main',
        ],
    },
)
