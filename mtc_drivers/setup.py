from setuptools import find_packages, setup

package_name = 'mtc_drivers'

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
    maintainer='anton',
    maintainer_email='moscowskyad@yandex.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_driver_node = mtc_drivers.sim_driver_node:main',
            'service = py_srvcli.service_member_function:main',
            'real_driver_node = mtc_drivers.real_robot_driver_node:main'
        ],
    },
)
