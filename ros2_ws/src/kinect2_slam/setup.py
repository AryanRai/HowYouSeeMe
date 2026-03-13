from setuptools import setup
import os
from glob import glob

package_name = 'kinect2_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Kinect2 SLAM with ORB-SLAM3 and TSDF integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tsdf_integrator = kinect2_slam.tsdf_integrator_node:main',
            'rerun_logger = kinect2_slam.rerun_logger_node:main',
        ],
    },
)
