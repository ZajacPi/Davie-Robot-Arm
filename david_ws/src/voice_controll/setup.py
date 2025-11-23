from setuptools import find_packages, setup

package_name = 'voice_controll'

setup(
    name=package_name,
    version='0.0.0',
packages=find_packages(include=['speech_recognition', 'speech_recognition.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/MovementCommand.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zajac',
    maintainer_email='piotr8zajac8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_commands_node = speech_recognition.voice_commands_node:main'
        ],
        
    },
)
