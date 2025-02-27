from setuptools import setup
import os
from glob import glob


package_name = 'tf_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
            'encoder = tf_encoder.tf_encoder:main',
            'sender = tf_encoder.tf_sender:main',
            'reciever = tf_encoder.tf_reciever:main',
            'odom_encoder = tf_encoder.tf_odom:main',
            'scan_encoder = tf_encoder.scan_encoder:main',
            'map_converter = tf_encoder.tf_encoder_map:main',
            'odom_converter = tf_encoder.tf_encoder_odom:main',
            'global_converter = tf_encoder.tf_globalFrameCalc:main',
        ],
    },
)
