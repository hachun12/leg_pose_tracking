from setuptools import find_packages, setup

package_name = 'leg_pose_camera'

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
    description='ZED 2i camera adapters and status helpers for leg pose tracking.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zed_status_node = leg_pose_camera.zed_status_node:main',
        ],
    },
)
