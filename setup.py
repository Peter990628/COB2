import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'hotel_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (
            os.path.join('share', package_name, 'models'),
            glob('models/*.pt')
        ),
        (
            os.path.join(
                "share", package_name, "calibration"),
            glob("calibration/*.npy"),
        ),





    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woods',
    maintainer_email='woods@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [

            'beverage_detector_node = hotel_vision.beverage_detector_node:main',
            'simple_movej_node = hotel_vision.simple_movej_node:main',
            'move_to_bev = hotel_vision.move_to_bev:main',
            'move_to_bev_opencv = hotel_vision.move_to_bev_opencv:main',
            'opencv_center_test = hotel_vision.opencv_center_test:main',
            'realsense_capture = train.realsense_dataset_capture_node:main',

        ],
    },
)
