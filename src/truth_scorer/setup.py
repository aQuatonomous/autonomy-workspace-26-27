from setuptools import find_packages, setup

package_name = 'truth_scorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/truth_scorer.yaml',
            'config/confusion_prior.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/truth_scorer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aQuatonomous',
    maintainer_email='dev@aquatonomous.ca',
    description='Calibrated existence, class and position confidence for mapped course objects.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'truth_scorer_node = truth_scorer.truth_scorer_node:main',
            'truth_eval_node = truth_scorer.truth_eval_node:main',
            'analyse_run = truth_scorer.analyse_run:main',
            'simulate_course = truth_scorer.simulate_course:main',
        ],
    },
)
