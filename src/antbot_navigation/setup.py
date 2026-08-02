from setuptools import find_packages, setup

package_name = 'antbot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/start_swarm_logic.launch.py',
            'launch/swarm.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tyashu',
    maintainer_email='tyashu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'swarm_logic = antbot_navigation.swarm_logic:main',
            'swarm_gui_server = antbot_navigation.swarm_gui_server:main'
        ],
    },
)
