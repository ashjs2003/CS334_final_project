from setuptools import setup

package_name = 'gita_waypoint_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/gita_waypoint_sim/launch', ['launch/gita_launch.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashjs',
    maintainer_email='ashjs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_ui_node = gita_waypoint_sim.waypoint_ui_node:main',
            'controller_node = gita_waypoint_sim.controller_node:main',
            'simulator_node = gita_waypoint_sim.simulator_node:main',
            'waypoint_listener = gita_waypoint_sim.waypoint_listener:main',
        ],
    },

)
