from glob import glob

from setuptools import find_packages, setup


package_name = 'hand_teleop'


# 모델 파일은 Git에 추가된 뒤 빌드하면 share/hand_teleop/models로 설치됩니다.
# 아직 모델을 내려받지 않은 상태에서도 패키지 빌드 자체는 가능하도록
# glob 결과가 있을 때만 data_files에 추가합니다.
data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    ('share/' + package_name, ['package.xml']),
]

model_files = glob('models/*.task')
if model_files:
    data_files.append(('share/' + package_name + '/models', model_files))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter-msi',
    maintainer_email='haebyuk35@gmail.com',
    description='MediaPipe hand tracking and Doosan robot teleoperation nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_tracker_node = hand_teleop.hand_tracker_node:main',
        ],
    },
)
