from setuptools import find_packages, setup

package_name = 'smartcar_scripts'

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
    maintainer='Fathym Diallo',
    maintainer_email='FK.Diallo@student.han.nl',
    description='Scripts package of smart_diffbot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = smartcar_scripts.joint_state_publisher:main',
            'joy_twist = smartcar_scripts.joy_twist:main',
            'vehicle_interface = smartcar_scripts.vehicle_interface:main',
            'wheel_odom = smartcar_scripts.wheel_odom:main',

        ],
    },
)
