from setuptools import find_packages, setup

package_name = 'carrot_trajectory_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'nav_msgs', 'mtc_msgs'],
    zip_safe=True,
    maintainer='banana',
    maintainer_email='banana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carrot_standalone = carrot_trajectory_following.carrot_standalone:main',
            'example_path = carrot_trajectory_following.example_path:main',
            'service = py_srvcli.service_member_function:main',
            'client = py_srvcli.client_member_function:main',
            'rviz2path_node = carrot_trajectory_following.rviz2path_node:main'
        ],
    },
)
