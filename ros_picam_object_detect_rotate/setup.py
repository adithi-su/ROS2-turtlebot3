from setuptools import find_packages, setup

package_name = 'object_detect_package'

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
    maintainer='aupadhya',
    maintainer_email='aupadhya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = object_detect_package.find_object:main",
	    "rotate_node = object_detect_package.rotate_object:main",
        ],
    },
)
