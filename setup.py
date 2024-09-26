import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'ira_collab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emma',
    maintainer_email='emma@brassville.com',
    description='ROS package for collaborative painting with an interactive, creative robotic arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'arm_node = ira_collab.arm_node:main',
        	'camera_node = ira_collab.camera_node:main',
            'eye_node = ira_collab.eye_node:main',
            'gpt_node = ira_collab.gpt_node:main',
            'interaction_node = ira_collab.interaction_node:main',
        ],
    },
)
