from setuptools import find_packages, setup

package_name = 'opencv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',"opencv-python"],
    zip_safe=True,
    maintainer='tung',
    maintainer_email='85216109+TomNgn3108@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera=opencv_pkg.camera_node:main'
        ],
    },
)
