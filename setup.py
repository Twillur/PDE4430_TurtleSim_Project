from setuptools import setup

package_name = 'turtlesim_pde4430'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='twillur',
    maintainer_email='WilliamKoju@gmail.com',
    description='TurtleSim movement patterns',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_line = turtlesim_pde4430.straight_line_node:main',
            'circle_movement = turtlesim_pde4430.circle_node:main',
            'figure_eight = turtlesim_pde4430.figure_eight_node:main',
            'roomba_cleaner = turtlesim_pde4430.roomba_node:main',
            'multi_turtle_node = turtlesim_pde4430.multi_turtle_node:main',
            'drive_user_input = turtlesim_pde4430.drive_user_input:main',
            'coordinate_navigation = turtlesim_pde4430.coordinate_navigation:main',
        ],
    },
)