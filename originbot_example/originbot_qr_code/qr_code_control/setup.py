from setuptools import setup
from glob import glob
package_name = 'qr_code_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件相关配置
        ('share/' + package_name, glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_code_control_node = qr_code_control.qr_code_control_node:main'
        ],
    },
)
