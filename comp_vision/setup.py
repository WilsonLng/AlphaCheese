from setuptools import find_packages, setup

package_name = 'comp_vision'

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
    maintainer='root',
    maintainer_email='stevennedwinn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testCam = comp_vision.testCam:main',
            'testCamModel = comp_vision.testCamModel:main',
            'chessVision = comp_vision.chessVision:main',
        ],
    },
)
