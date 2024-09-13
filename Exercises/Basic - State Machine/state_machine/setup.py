from setuptools import find_packages, setup

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tung',
    maintainer_email='85216109+TomNgn3108@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = state_machine.state_client:main',
            'server = state_machine.state_server:main',
            'sub_node = state_machine.node:main'
        ],
    },
)
