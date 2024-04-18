from setuptools import find_packages, setup

package_name = 'ira'

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
    maintainer='emma',
    maintainer_email='emma@brassville.com',
    description='ROS package for an interactive, creative robotic arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = ira.publisher_member_function:main',
        	'listener = ira.subscriber_member_function:main',
        ],
    },
)
