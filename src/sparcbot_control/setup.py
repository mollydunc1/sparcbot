from setuptools import find_packages, setup

package_name = 'sparcbot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SPARCBot',
    maintainer_email='sparcbot@example.com',
    description='SPARCBot controllers in Python.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_controller = sparcbot_control.wheel_controller_node:main',
            'linear_vel_step_test = sparcbot_control.linear_vel_step_test:main',
            'angular_vel_step_test = sparcbot_control.angular_vel_step_test:main',
        ],
    },
)
