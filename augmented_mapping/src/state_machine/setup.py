from setuptools import find_packages, setup

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/NotifyPositionReached.srv', 'srv/StartDetection.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunhajo',
    maintainer_email='yjo@caltech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine_node = state_machine.state_machine:main'
        ],
    },
)
