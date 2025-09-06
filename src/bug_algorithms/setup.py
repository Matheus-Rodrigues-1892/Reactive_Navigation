from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bug_algorithms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matheus',
    maintainer_email='matheus@todo.todo',
    description='Pacote de algoritmos Bug para ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_node = bug_algorithms.bug1_node:main',
            'publisher = bug_algorithms.publisher:main',
            'subscriber = bug_algorithms.subscriber:main',
            'navigation = bug_algorithms.bug1_node:main',
            'bug1_node = bug_algorithms.bug1_node:main',
            'bug_tangent =  bug_algorithms.bug_tangent:main',
        ],
    },
)
