from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'tiago_moveitpy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yoan Mollard',
    maintainer_email='opensource@aubrune.eu',
    description='MoveItPy demonstrator for Tiago on Gazebo Harmonic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick = tiago_moveitpy.pick:main'
        ],
    },
)
