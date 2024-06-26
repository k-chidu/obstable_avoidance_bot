from setuptools import find_packages, setup

package_name = 'sensor'

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
    maintainer='raspi',
    maintainer_email='raspi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'middle_sensor = sensor.US_middle:main',
            'left_sensor = sensor.US_left:main',
            'right_sensor = sensor.US_right:main',
            'motor_driver = sensor.motor_control:main'
        ],
    },
)
