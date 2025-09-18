from setuptools import find_packages, setup

package_name = 'david_robot_arm_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zajac',
    maintainer_email='zajac@todo.todo',
    description='code for a 6DOF robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = david_robot_arm_pkg.my_node:main',
            'controller_evdev_node = david_robot_arm_pkg.controller_test_node:main',
            'receiver_node = david_robot_arm_pkg.receiver_node:main'
        ],
    },
)
