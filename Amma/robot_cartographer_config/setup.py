import os
from glob import glob
from setuptools import setup

package_name = 'robot_cartographer_config' # Ensure this matches your package name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # This usually points to the python module, if you have one
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install your launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # If you also have a config directory with .lua or .yaml files that Cartographer needs directly from share:
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # Update this
    maintainer_email='your.email@example.com', # Update this
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any executable nodes here if you have them, e.g.:
            # 'my_node = robot_cartographer_config.my_node:main'
        ],
    },
)
