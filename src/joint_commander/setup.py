from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joint_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='malattar',
    maintainer_email='mohamadalattar1999@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_joint_angles = joint_commander.publish_joint_angles:main',
            'joint_publisher = joint_commander.task_1:main',
        ],
    },
)