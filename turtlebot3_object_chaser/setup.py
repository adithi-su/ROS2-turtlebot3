from setuptools import find_packages, setup

package_name = 'chaser'

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
    maintainer='burger',
    maintainer_email='anirudhtulasi2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
          "detect_node_exec = chaser.detect_object:main",
	      "range_node_exec = chaser.get_object_range:main",
          "chase_node_exec = chaser.chase_object:main",
        ],
    },
)
