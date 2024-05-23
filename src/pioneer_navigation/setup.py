import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'pioneer_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
         
         (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joacorn',
    maintainer_email='joaquin.cornejo@utec.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager = pioneer_navigation.manager:main',
            'markers = pioneer_navigation.markers:main',
            'objects = pioneer_navigation.objects:main',
            'navigator = pioneer_navigation.navigator:main',
            'navigator_back = pioneer_navigation.navigator_back:main',
        ],
    },
)
