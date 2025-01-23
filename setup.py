from setuptools import find_packages, setup
from glob import glob

package_name = 'msiclaw_teleop_twist_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msiclaw',
    maintainer_email='msiclaw@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msiclaw_joy = msiclaw_teleop_twist_joy.msiclaw_joy:main',
            'msiclaw_joy_teleop_manager = msiclaw_teleop_twist_joy.msiclaw_joy_teleop_manager:main'
        ],
    },
)
