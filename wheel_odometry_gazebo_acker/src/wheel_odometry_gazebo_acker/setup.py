from setuptools import setup

package_name = 'wheel_odometry_gazebo_acker'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@youremail.com',
    maintainer='Mahmoud Tahmasebi',
    maintainer_email='your@youremail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A simple ROS2 Python package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_wheel_odometry_gazebo_acker = wheel_odometry_gazebo_acker.publish_wheel_odometry_gazebo_acker:main'
        ],
    },

    data_files=[
        # This will copy package.xml into share folder
        ('share/ament_index/resource_index/packages', ['package.xml']),
        # This will copy your resource folder to the share folder
        ('share/' + package_name, ['resource/' + package_name]),
        # This will copy your launch file into the share/launch directory
        ('share/' + package_name + '/launch', ['launch/publish_wheel_odometry_gazebo_acker_launch.py']),
    ],
)
