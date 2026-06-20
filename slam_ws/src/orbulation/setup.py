from setuptools import find_packages, setup
from glob import glob

package_name = 'orbulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mt-labpc',
    maintainer_email='{hironmoy.roy.rudra@g.bracu.ac.bd}',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test = orbulation.orbslam3_rgbd_ros2:main',
            'orb_slam_rgbd = orbulation.orb_slam_rgbd:main',
        ],
    },
)
