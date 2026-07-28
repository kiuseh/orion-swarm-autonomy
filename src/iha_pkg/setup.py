from setuptools import find_packages, setup

package_name = 'iha_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Hüseyin Sefa Kiriş',
    maintainer_email='kiris.h.sefa@gmail.com',
    description='Mission, formation, and MAVSDK flight control for the Orion swarm',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "uav_control = iha_pkg.uav_control:main",
            "temp = iha_pkg.temp_info_sub:main",
        ],
    },
)
