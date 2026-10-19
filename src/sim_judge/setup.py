from setuptools import find_packages, setup

package_name = 'sim_judge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aQuatonomous',
    maintainer_email='aquatonomous@queensu.ca',
    description='Simulated RoboBoat 2026 judge (RoboCommand stand-in + referee).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'judge = sim_judge.judge:main',
            'send_test_reports = sim_judge.send_test_reports:main',
        ],
    },
)
