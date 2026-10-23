from setuptools import find_packages, setup

package_name = 'sparcbot_teleop'

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
    description='Keyboard teleop for SPARCBot.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = sparcbot_teleop.keyboard_teleop:main',
            'pwm_teleop = sparcbot_teleop.pwm_teleop:main',
            'bump_teleop = sparcbot_teleop.bump_teleop:main'
        ],
    },
)
