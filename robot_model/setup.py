from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), glob('resource/' + package_name)),
        (os.path.join('share', package_name), glob('package.xml')),
        # Install LAUNCH files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        
        # Install RViz configuration files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Install  configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install map files
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
