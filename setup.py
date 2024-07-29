from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'calibration_tool'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='realsense',
    maintainer_email='kisho@ucl.nuee.nagoya-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_center = calibration_tool.find_center:main',
            'tf_table = calibration_tool.tf_table:main',
            'find_marker = calibration_tool.find_marker:main',
            'find_table_location = calibration_tool.find_table_location:main',
        ],
    },
)
