from setuptools import find_packages, setup

package_name = 'cv_lidar_fusion'

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
    maintainer='lorenzo',
    maintainer_email='lorenzodemarni666@gmail.com',
    description='Vision-LiDAR fusion: color from CV into LiDAR tracked buoys.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_lidar_fusion = cv_lidar_fusion.vision_lidar_fusion:main',
        ],
    },
)
