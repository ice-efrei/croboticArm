from setuptools import find_packages, setup

package_name = 'ros_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
('share/' + package_name + '/launch', ['launch/arm.launch.py']),    
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crobotic',
    maintainer_email='crobotic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'arm_node = ros_arm.arm_node:main',
        'test_loop = ros_arm.test_loop:main',
    ],
},

 package_data={
        '': ['srv/*.srv'],
    },
)
