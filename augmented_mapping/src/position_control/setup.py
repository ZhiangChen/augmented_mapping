from setuptools import find_packages, setup


package_name = 'position_control'


setup(
<<<<<<< HEAD
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
       ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/srv', ['srv/SetTargetPosition.srv'])
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
           'position_control = position_control.position_control:main',
           'position_control_final = position_control.position_control_sm:main'
       ],
   },
=======
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/SetTargetPosition.srv'])
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
            'position_control = position_control.position_control:main',
            'position_control_final = position_control.position_control_sm:main'
        ],
    },
>>>>>>> 1babdd5953934a3ac50ce7524e2f58a29a70478a
)





