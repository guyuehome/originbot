from setuptools import setup
from glob import glob
import os

package_name = 'originbot_vlpr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name, 'vlpt_package'), glob('originbot_vlpr/vlpt_package/*.*')),
        (os.path.join('lib', package_name, 'vlpt_package/crnn'), glob('originbot_vlpr/vlpt_package/crnn/*.*')),
        (os.path.join('lib', package_name, 'vlpt_package/dbnet'), glob('originbot_vlpr/vlpt_package/dbnet/*.*')),
        (os.path.join('lib', package_name, 'vlpt_package/utils'), glob('originbot_vlpr/vlpt_package/utils/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='TODO@TODO.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlpr_node = originbot_vlpr.vlpr_node:main'
        ],
    },
)
