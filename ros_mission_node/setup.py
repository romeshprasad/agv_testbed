from setuptools import setup

package_name = 'ros_mission_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Testbed',
    maintainer_email='user@example.com',
    description='ROS2 mission dispatch node for AGV Level 2 testbed',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = ros_mission_node.mission_node:main',
        ],
    },
)
