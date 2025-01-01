from setuptools import find_packages, setup

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='susheel',
    maintainer_email='bsusheelkumar01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_server = sensor_interface.sensor_server:main',
            'sensor_client = sensor_interface.sensor_client:main'
        ],
    },
)