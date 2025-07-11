from setuptools import find_packages, setup

package_name = 'ydlidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ydlidar_launch.py']),
        ('share/' + package_name + '/params', ['params/ydlidar.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ydlidar_node = ydlidar.ydlidar_node:main',
            'ydlidar_node_socket = ydlidar.ydlidar_node_socket:main',
        ],
    },
)
