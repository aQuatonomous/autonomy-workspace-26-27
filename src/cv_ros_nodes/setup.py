from setuptools import find_packages, setup

package_name = 'cv_ros_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Keep the canonical mapping file under computer_vision/ (repo root).
        ('share/' + package_name, ['../../computer_vision/class_mapping.yaml']),
        ('share/' + package_name + '/launch',
        ['cv_ros_nodes/launch/launch_cv.py', 'cv_ros_nodes/launch/launch_cv_sim.py', 'cv_ros_nodes/launch/launch_cv_single_camera1.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzodemarni666@gmail.com',
    description='ROS 2 nodes for computer vision: preprocessing, inference, combiner, and task processors.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_preprocessing = cv_ros_nodes.vision_preprocessing:main',
            'vision_inference = cv_ros_nodes.vision_inference:main',
            'vision_combiner = cv_ros_nodes.vision_combiner:main',
            'camera_viewer = cv_ros_nodes.camera_viewer:main',
            'task4_supply_processor = cv_ros_nodes.task4_supply_processor:main',
            'indicator_buoy_processor = cv_ros_nodes.indicator_buoy_processor:main',
            'maritime_distance_estimator = cv_ros_nodes.maritime_distance_estimator:main',
        ],
    },
)
