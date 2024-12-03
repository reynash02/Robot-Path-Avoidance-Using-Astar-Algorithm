from setuptools import find_packages, setup

package_name = 'path_avoidance'

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
    maintainer='reynash',
    maintainer_email='yashnair7779@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'gui_sim=path_avoidance.turtle_gui_simulation:main',
        	'obstacle_marking=path_avoidance.obstacle_marking:main',
        ],
    },
)
