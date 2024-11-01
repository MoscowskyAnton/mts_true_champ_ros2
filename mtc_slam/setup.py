from setuptools import find_packages, setup

package_name = 'mtc_slam'

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
    maintainer='Moscowsky Anton',
    maintainer_email='moscowskyad@yandex.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_srvcli.service_member_function:main',
            #'slam_node = mtc_slam.slam_node:main',
            'step_slam_node = mtc_slam.step_slam_node:main',
            'client = py_srvcli.client_member_function:main',
            'bag_play_node = mtc_slam.bag_play_node:main',
            'bag_plug_node = mtc_slam.bag_play_plug_node:main'
        ],
    },
)
