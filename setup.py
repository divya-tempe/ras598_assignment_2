from setuptools import setup
from glob import glob
import os

package_name = 'ras598_assignment_2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    scripts=['grading_scout.py'],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml', 'map.yaml', 'cave_filled.png']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'Pillow',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 planner package for RAS598 Assignment 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = ras598_assignment_2.planner_node:main',
        ],
    },
)
