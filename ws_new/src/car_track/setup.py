from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_track'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')), 
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
        (os.path.join('share', package_name, 'map'), glob('map/*.*')),      # 이 파일들이 BUILD폴더로 넘어감!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kante',
    maintainer_email='94979429+leekante@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
