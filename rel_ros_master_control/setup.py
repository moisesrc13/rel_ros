from setuptools import find_packages, setup

package_name = 'rel_ros_master_control'

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
    maintainer='relant',
    maintainer_email='relant@relant.com',
    description='Master control for pumps',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rel_ros_master_control_node = rel_ros_master_control.index:main"
        ],
    },
)
