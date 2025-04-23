from setuptools import setup
import os
from glob import glob

package_name = 'auto_waffle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='abdul',
    maintainer_email='abdulwakil.ola@gmail.com',
    description='ROS2 node for controlling the Waffle robot with motor HAT and encoders',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waffle_bot = auto_waffle.waffle_bot:main'
        ],
    },
    install_requires=[
        'setuptools',
        'pyserial',
        'Adafruit-MotorHAT',
        'transforms3d',
    ],
)
