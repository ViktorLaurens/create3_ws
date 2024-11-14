from setuptools import find_packages, setup

package_name = 'create3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/move_to_goal.launch.py']),
        ('share/' + package_name, ['config/target_position.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viktordg',
    maintainer_email='viktor.degroote@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_goal = create3_control.move_to_goal:main',
        ],
    },
)
