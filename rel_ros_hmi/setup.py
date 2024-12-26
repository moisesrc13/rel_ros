from setuptools import find_packages, setup

package_name = 'rel_ros_hmi'

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
    description='HMI node with publisher and subscriber for messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rel_ros_hmi_node = rel_ros_hmi.index:main"
        ],
    },
)
