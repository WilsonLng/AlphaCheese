from setuptools import find_packages, setup

package_name = 'motion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='rostwotesting',
    maintainer_email='wilsonzzz19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_write_node = motion_pkg.read_write_node:main',
            'angles_producer = motion_pkg.angles_producer:main',
            'mover = motion_pkg.mover:main',
            'moveAndTake = motion_pkg.moveAndTake:main'
        ],
    },
)
