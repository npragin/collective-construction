from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'scouts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liam-bouffard',
    maintainer_email='liamtbo@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner_node = scouts.path_planner:main',
            'robot_tf = scouts.robot_tf:main',
            'odom2base_tf = scouts.odom2base_tf:main'
        ],
    },
)
