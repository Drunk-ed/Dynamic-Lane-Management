from setuptools import find_packages, setup
import os

package_name = 'lane_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    (
        os.path.join('share', package_name, 'launch'),
        ['launch/two_robot.launch.py',
         'launch/two_robot_nav.launch.py',
         'launch/demo.launch.py',
         'launch/cloverleaf_world.launch.py',
         'launch/spawn_divider.launch.py',
         'launch/cloverleaf_nav2.launch.py',
         'launch/divider_bringup.launch.py',
         'launch/bringup_launch.py',
         'launch/controller_only.launch.py',
         'launch/rpp_controller.launch.py',],
    ),
    
    (
	    os.path.join("share", package_name, "param"),
	    ["param/nav2_multirobot.yaml"],
	),
	],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t-sim',
    maintainer_email='t-sim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
