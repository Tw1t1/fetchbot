from setuptools import find_packages, setup

package_name = 'obstacle_avoidance'

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
    maintainer='ben',
    maintainer_email='Ben.shervi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
            'feel_force = obstacle_avoidance.feel_force:main',
            'runaway = obstacle_avoidance.runaway:main',
            'avoid_runaway_suppressor = obstacle_avoidance.avoid_runaway_suppressor:main',
            'indicator = obstacle_avoidance.indicator:main',
            'locomotion_controller = obstacle_avoidance.locomotion_controller:main',
            'bumper = obstacle_avoidance.bumper:main',
        ],
    },
)
