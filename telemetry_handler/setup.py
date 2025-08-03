from setuptools import find_packages, setup

package_name = 'telemetry_handler'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ceccon',
    maintainer_email='ceccon@example.com',
    description='Telemetry handling, visualization and recording package for drone systems',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_handler = telemetry_handler.telemetry_handler:main',
            'log_viewer = telemetry_handler.log_viewer:main',
            'telemetry_recorder = telemetry_handler.telemetry_recorder:main',
            'telemetry_dashboard = telemetry_handler.telemetry_dashboard:main'
        ],
    },
)
