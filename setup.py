from setuptools import setup
import os
from glob import glob

package_name = 'spider_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='spider bot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spider_bot_control = spider_bot.spider_bot_control:main',
        ],
    },
)
