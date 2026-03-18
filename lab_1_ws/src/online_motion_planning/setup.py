from setuptools import find_packages, setup

package_name = 'online_motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/online_motion_planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giang',
    maintainer_email='giang.nht108201@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_tb = online_motion_planning.control_tb:main',
            'grid_mapping = online_motion_planning.grid_mapping:main',
            'localization_node = online_motion_planning.localization_node:main',
            'online_motion_planning_node = online_motion_planning.online_motion_planning_node:main',
        ],
    },
)
