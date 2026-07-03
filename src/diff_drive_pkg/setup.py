import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'diff_drive_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),

        # Install Config Files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Install URDF/XACRO Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # Install Worlds Files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Differential Drive Robot Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_switcher = diff_drive_pkg.lane_switcher:main',        ],
    },
)