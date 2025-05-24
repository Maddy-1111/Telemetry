from setuptools import find_packages, setup

package_name = 'uplink_package'

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
    maintainer='madhav',
    maintainer_email='madhav.tadepalli11@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    package_data={'uplink_package': ['packet_structure.json']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'uplink_node = uplink_package.uplink_node:main'
        ],
    },
)
