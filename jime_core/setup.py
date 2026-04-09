from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'jime_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'ui'), glob('ui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team204',
    maintainer_email='aemil072@uottawa.ca',
    description='Jim-E Robot Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'brain = jime_core.host_robot_logic:main',
            'ultrasonic = jime_core.ultrasonic_driver:main',
            'vision = jime_core.vision_node:main',
            'streamer = jime_core.web_streamer:main',
            'start_bridge = jime_core.start_bridge:main',
            'led_commander = jime_core.led_commander:main',
            'bridge_node = jime_core.esp32_bridge:main',
        ],
    },
)
