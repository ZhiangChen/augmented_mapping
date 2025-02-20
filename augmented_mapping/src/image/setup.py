from setuptools import find_packages, setup

package_name = 'image'

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
    maintainer='yunhajo',
    maintainer_email='yjo@caltech.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_node = image.image_node:main',
            'image_pc_node = image.image_pc_node:main'
            'image_bbox_node = image.image_bbox_node:main'
        ],
    },
)
