from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obj_detection_node'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name + '/models'), glob('models/*.pt')),
        (os.path.join('share', package_name + '/util'), glob('util/*.py')),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peoples',
    maintainer_email='Atma1',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_publisher = obj_detection_node.publish_dectection:main',
            'localization_publisher = obj_detection_node.publish_localization:main'
        ],
    },
)
