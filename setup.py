from setuptools import find_packages, setup

package_name = 'msiclaw_teleop_twist_joy'

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
    maintainer='msiclaw',
    maintainer_email='msiclaw@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msiclaw_teleop_twist_joy = msiclaw_teleop_twist_joy.msiclaw_teleop_twist_joy:main',
        ],
    },
)
