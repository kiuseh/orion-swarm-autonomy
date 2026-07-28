from setuptools import find_packages, setup


package_name = 'g_isleme_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: ['models/wechat_qr/*'],
        package_name + '.models.wechat_qr': ['*.prototxt', '*.caffemodel'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Hüseyin Sefa Kiriş',
    maintainer_email='kiris.h.sefa@gmail.com',
    description='Image processing package for swarm workspace',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'udp_img_prc = g_isleme_pkg.udp_img_prc:main',
        ],
    },
)
