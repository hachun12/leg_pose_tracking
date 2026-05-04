from setuptools import find_packages, setup

package_name = 'leg_pose_openpose'

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
    description='OpenPose adapter nodes for 2D leg keypoint detection and overlays.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'openpose_node = leg_pose_openpose.openpose_node:main',
        ],
    },
)
