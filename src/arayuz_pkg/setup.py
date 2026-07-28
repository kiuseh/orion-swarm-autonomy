from setuptools import find_packages, setup

package_name = 'arayuz_pkg'
package_files = [
    '*.ui',
    '*.png',
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: package_files,
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Hüseyin Sefa Kiriş',
    maintainer_email='kiris.h.sefa@gmail.com',
    description='Ground control interface package for the swarm workspace',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interface = arayuz_pkg.qgc_node2:main',
            'qgc_interface = arayuz_pkg.qgc_node2:main',
            'legacy_interface = arayuz_pkg.arayuz_islem:main',
            'qgc_legacy_interface = arayuz_pkg.qgc_node:main',
            'yki_drone_test = arayuz_pkg.yki_drone_test_node:main',
        ],
    },
)
