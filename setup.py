from setuptools import find_packages, setup

package_name = 'vdcs_receiver_node'

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
    maintainer='osboxes',
    maintainer_email='osboxes@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vdcs_receiver_node_ser = vdcs_receiver_node.vdcs_receiver:main_ser',
            'vdcs_receiver_node_udp = vdcs_receiver_node.vdcs_receiver:main_udp',
        ],
    },
)
