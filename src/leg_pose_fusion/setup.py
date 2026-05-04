from setuptools import find_packages, setup

package_name = 'leg_pose_fusion'

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
    maintainer='hachun',
    maintainer_email='hachun@todo.todo',
    description='3D keypoint fusion and leg joint angle estimation nodes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keypoint_fusion_node = leg_pose_fusion.keypoint_fusion_node:main',
            'angle_estimator_node = leg_pose_fusion.angle_estimator_node:main',
            'synthetic_keypoints_node = leg_pose_fusion.synthetic_keypoints_node:main',
        ],
    },
)
