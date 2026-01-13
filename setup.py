from setuptools import find_packages, setup

package_name = 'my_pi_nodes'

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
    maintainer='benja',
    maintainer_email='benja@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = my_pi_nodes.controller:main',
            'serial_bridge = my_pi_nodes.serial_bridge:main',
            'MotorCmd = my_pi_nodes.MotorCmd:main',
            'Control = my_pi_nodes.Control:main',
            'tip_plotter = my_pi_nodes.tip_plotter:main',
            'MotorCmd_Test = my_pi_nodes.MotorCmd_Test:main',
            'Control_Test = my_pi_nodes.Control_Test:main',
        ],
    },
)
