from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mars_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch','nav2_bringup'), glob('launch/nav2_bringup/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')), 
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle_pi'), glob('models/turtlebot3_waffle_pi/*.*')),       
        (os.path.join('share', package_name, 'params'), glob('params/*.*')),       
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),       
        (os.path.join('share', package_name, 'map'), glob('map/*.*')),       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunwolee',
    maintainer_email='128200788+malenwater@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
