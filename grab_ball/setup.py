from setuptools import find_packages, setup

package_name = 'grab_ball'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'hardware_librery'],
    zip_safe=True,
    maintainer='Yosef',
    maintainer_email='yosefseada@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'claw_controller = grab_ball.claw_controller:main',
            'locomotion_controller = grab_ball.locomotion_controller:main',
            'grab_ball = grab_ball.ball_grabber:main',
        ],
    },
)
