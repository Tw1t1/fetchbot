from setuptools import find_packages, setup

package_name = 'fetchbot_testing'

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
    maintainer='yosef',
    maintainer_email='shalomfall@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_mover = fetchbot_testing.ball_mover:main',
            'twist_publisher = fetchbot_testing.heading_to_twist_pub:main',
            'test_ball_detection = fetchbot_testing.test_ball_detection:main',
            'simulated_claw_controller = fetchbot_testing.simulated_claw_controller:main',
        ],
    },
)
