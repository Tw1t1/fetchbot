from setuptools import setup
from glob import glob

package_name = 'ball_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='yosefseada@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ball = ball_follow.detect_ball:main',
            'detect_ball_3d = ball_follow.detect_ball_3d:main',
            'follow_ball = ball_follow.follow_ball:main',
            'orient_home_ball_detection_inhibitor = ball_follow.inhibit_detect_ball:main',
            'grab_ball_follow_ball_inhibitor = ball_follow.inhibit_follow_ball:main',
            'camera = ball_follow.camera:main',
        ],
    },
)
