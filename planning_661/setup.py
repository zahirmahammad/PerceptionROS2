from setuptools import find_packages, setup

package_name = 'planning_661'

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
    maintainer='zahir',
    maintainer_email='zahirmd1604@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mycontrol = planning_661.mycontrol:main',
            'printodom = planning_661.printodom:main',
            'publishodom = planning_661.publishodom:main'
        ],
    },
)
