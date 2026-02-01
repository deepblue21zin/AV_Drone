from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mppi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        # ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        # package.xml
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='quddnr',
    maintainer_email='quddnr@todo.todo',

    description='MPPI offboard orchestrator (state-based, modular)',
    license='TODO',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            # 큰 틀 (상태 머신 오케스트레이터)
            'mppi_node = mppi.mppi_node:main',
        ],
    },
)
