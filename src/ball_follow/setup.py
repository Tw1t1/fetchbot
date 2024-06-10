from setuptools import find_packages, setup

package_name = 'ball_follow'

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
            "video_publisher = ball_follow.usb_camera:main",
            "follow_ball = ball_follow.follow_ball:main",
            "detect_ball = ball_follow.detect_ball:main"            
        ],
    },
)
