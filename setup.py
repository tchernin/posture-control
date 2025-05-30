import os
from setuptools import setup
from typing import List

package_name = 'posture_control'

def get_files(dir_name: str) -> List[str]:
    return [
        os.path.join(root, file)
        for root, _, files in os.walk(dir_name)
        for file in files
    ]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', get_files('config')),
        ('share/' + package_name + '/trained_models', get_files('trained_models')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Celine Tchernin',
    maintainer_email='you@example.com',
    description='Whill control using Sensor mats.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cop_posture_control = posture_control.cop_posture_control:main',
            'model_based_posture_control = posture_control.trained_model_posture_control:main',
        ],
    },
)
