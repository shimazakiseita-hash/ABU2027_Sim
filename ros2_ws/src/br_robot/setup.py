from setuptools import find_packages, setup

package_name = 'br_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seita Shimazaki',
    maintainer_email='shimazakiseita@gmail.com',
    description='ABU Robocon 2027 TR/BR: 実機でもシムでも同じものを動かす足回り制御・推測航法・機構制御のノード',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'base_controller_node = br_robot.base_controller_node:main',
            'odometry_node = br_robot.odometry_node:main',
            'mechanism_controller_node = br_robot.mechanism_controller_node:main',
        ],
    },
)
