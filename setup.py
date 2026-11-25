from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'kh_disparity_extender'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karlo Habek',
    maintainer_email='karlo.habek@fsb.hr',
    description='Disparity Extender obstacle avoidance and reactive navigation for F1TENTH / RoboRacer (PAS coursework).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'disparity_extender = kh_disparity_extender.disparity_extender_node:main',
        ],
    },
)
