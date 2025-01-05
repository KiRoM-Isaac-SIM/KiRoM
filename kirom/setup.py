from setuptools import setup, find_packages

package_name = 'kirom'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'launch/*.py',
            'package.xml',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeonsu',
    maintainer_email='hyeonsu@todo.todo',
    description='Hello MoveIt package for controlling a robot using MoveIt in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'hello_node = kirom.some_module:main'
        ],
    },
)
