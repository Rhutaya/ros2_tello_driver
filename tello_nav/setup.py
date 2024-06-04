import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tello_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthurboutignon@gmail.com',
    description='Package handle navigation order to send it to action server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_nav = tello_nav.tello_nav_handle:main',
            'pub_only = tello_nav.simple_pub:main',
        ],
    },
)
