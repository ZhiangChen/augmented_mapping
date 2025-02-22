from setuptools import find_packages, setup

package_name = 'velocity_control'

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
    maintainer='y2204',
    maintainer_email='yjo@caltech.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_control_node = velocity_control.velocity_control:main',
            'velocity_manual_node = velocity_control.velocity_manual_control:main',
            'velocity_fuel_node = velocity_control.velocity_fuel_control:main',
            'velocity_bbox_node = velocity_control.velocity_bbox_control:main',
        ],
    },
)
