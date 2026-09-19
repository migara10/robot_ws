import os
from glob import glob
from setuptools import setup

package_name = 'arm_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='migara',
    maintainer_email='migara@todo.todo',
    description='Dummy pick-and-place demo using MoveItPy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place_demo = arm_pick_place.pick_place_demo:main',
        ],
    },
)