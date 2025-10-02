from setuptools import setup

package_name = 'p2p_controller'

setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'move = p2p_controller.move:main',
        ],
    },
)