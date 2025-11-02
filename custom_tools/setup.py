from setuptools import find_packages, setup

package_name = 'custom_tools'

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
    maintainer='nick',
    maintainer_email='skrillx789@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'toggle_time_node = custom_tools.toggle_time_node:main',
            'night_vision_camera_node = custom_tools.night_vision_camera_node:main',
            'cloud_accumulator = custom_tools.cloud_accumulator:main'

        ],
    },
)
