from setuptools import find_packages, setup

package_name = 'sim_autolabel'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['classes.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aQuatonomous',
    maintainer_email='aquatonomous@queensu.ca',
    description='Labelled CV training images from the simulator (recorder, YOLO/Roboflow export and upload).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = sim_autolabel.record:main',
            'export = sim_autolabel.export:main',
            'upload = sim_autolabel.upload:main',
        ],
    },
)
