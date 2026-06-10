import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jn2-alt',
    maintainer_email='jar3dnorth@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_aruco_test = manipulator.rgb_aruco_test:main',
            'absolute_move = manipulator.absolute_move:main',
            'pick = manipulator.pick:main',
            'fsm = manipulator.fsm:main',
            'correction_task_server = manipulator.correction_task_server:main',
            'placement_accuracy_checker = manipulator.placement_accuracy_checker:main',
            'test_fsm = manipulator.test_fsm:main',
            'manipulation_server = manipulator.manipulation_server:main',
            'blockscan = manipulator.blockscan:main'
        ],
    },
)
