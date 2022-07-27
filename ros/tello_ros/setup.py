from setuptools import setup

package_name = 'tello_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='root@todo.todo',
    description='Using ros with tello drones',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_simple_solution = tello_ros.tello_simple_solution:main',
            'image_processing = tello_ros.image_processing:main',
            'marker_follower = tello_ros.aruco_marker_follower:main',
            'manager_node = tello_ros.manager_node:main',
            'middleware_joy_node = tello_ros.middleware_joy_node:main'
        ],
    },
)
