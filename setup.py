from setuptools import find_packages, setup

package_name = 'cobot_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peternote',
    maintainer_email='haebyuk35@gmail.com',
    description='cobot2_project',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_test = peter.gripper_test:main',
            'beverage_test = peter.beverage_test:main',
            'go_home = peter.go_home:main',
            'jog_complete = peter.jog_complete:main',
            'get_current_pose = peter.get_current_pose:main',
        ],
    },
)
