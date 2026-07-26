from setuptools import setup
import os
from glob import glob

package_name = 'io_mocap_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='io_zhaofy',
    maintainer_email='zhaofy@io-intelligence.com',
    description='Visualize blender_human urdf with external tf',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_relay = io_mocap_visualization.tf_relay:main',
        ],
    },
)
