from setuptools import find_packages, setup

package_name = 'br_strategy_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_simulator.py']),
    ],
    install_requires=['setuptools', 'pygame', 'pymunk'],
    zip_safe=True,
    maintainer='Seita Shimazaki',
    maintainer_email='shimazakiseita@gmail.com',
    description='ABU Robocon 2027 BR自律システム Phase 1: 2Dシミュレータ (pymunk + pygame)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'br_sim_bridge_node = br_strategy_sim.sim_bridge_node:main',
            'br_referee_node = br_strategy_sim.br_referee_node:main',
            'br_observation_node = br_strategy_sim.br_observation_node:main',
            'br_visualizer_node = br_strategy_sim.br_visualizer_node:main',
            'br_decision_tr_node = br_strategy_sim.br_decision_tr_node:main',
            'br_decision_br_node = br_strategy_sim.br_decision_br_node:main',
        ],
    },
)
