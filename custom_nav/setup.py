from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'custom_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='csl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'nav_id2id_node = custom_nav.nav_id2id:main',
            'client = custom_nav.srv_client_test:main',
            # 'nav_act = custom_nav.nav_act:main',
            'nav_srv_node = custom_nav.nav_srv:main', 
        ],
    },
)
