from setuptools import find_packages, setup

package_name = 'leg_pose_gui'

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
    description='GUI and status display tools for leg pose tracking.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status_gui_node = leg_pose_gui.status_gui_node:main',
            'topic_monitor_node = leg_pose_gui.topic_monitor_node:main',
            'qt_gui_node = leg_pose_gui.qt_gui_node:main',
        ],
    },
)
