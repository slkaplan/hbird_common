from setuptools import setup
import os
from glob import glob

package_name = 'hbird_navigation'
submodules = 'hbird_navigation/scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olin-hair-lab',
    maintainer_email='kene.mbanisi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_control_node = hbird_navigation.agent_control_node:main'
        ],
    },
)
