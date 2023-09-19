from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'command_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tariqul',
    maintainer_email='taricool.islam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "duty_control=command_mapper.duty_control:main",
            "rpm_control=command_mapper.rpm_control:main",
            "const_cmd_vel=command_mapper.const_cmd_vel:main"
        ],
    },
)
