from setuptools import find_packages, setup

package_name = 'wander'

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
    entry_points={
        'console_scripts': [
            'wander = wander.wander:main',
            'avoid = wander.avoid:main',
            'ball_deatection_wander_inhibitor = wander.ball_deatection_wander_inhibitor:main',
            'follow_ball_wander_suppressor = wander.follow_ball_wander_suppressor:main',
            'orient_home_wander_suppressor = wander.orient_home_wander_suppressor:main',
        ],
    },
)
