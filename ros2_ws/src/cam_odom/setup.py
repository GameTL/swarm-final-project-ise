from setuptools import find_packages, setup

package_name = 'cam_odom'
# submodules = package_name +'/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='limsila.limsila@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = cam_odom.cam_odom_client:main',
        ],
    },
)
