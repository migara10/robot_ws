import os
from glob import glob
from setuptools import setup

package_name = 'arm_hardware_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Real hardware bridge connecting MoveIt2 to ESP32 servo controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hardware_bridge_node = arm_hardware_bridge.hardware_bridge_node:main',
        ],
    },
)