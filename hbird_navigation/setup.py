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
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'agent_control = hbird_navigation.agent_control_node:main',
            'crazyflie_viz = hbird_navigation.crazyflie_viz_node:main',
            'crazyflie_sim = hbird_navigation.crazyflie_sim_node:main',
            'crazyflie_hdw = hbird_navigation.crazyflie_hdw_node:main',
            'ground_control = hbird_navigation.ground_control_node:main'
        ],
    },
)
