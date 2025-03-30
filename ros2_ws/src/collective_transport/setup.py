from setuptools import setup

package_name = 'collective_transport'
submodules = package_name +'/submodules'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    # data_files=[
    #     ('share/ament_index/resource_index/packages'),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for x-drive robot control and logging',
    license='MIT License',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collective_transport = collective_transport.collective_transport:main',
        ],
    },
)