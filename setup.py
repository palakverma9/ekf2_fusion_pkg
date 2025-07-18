from setuptools import setup

package_name = 'ekf2_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Palak',
    maintainer_email='pglitch89@gmail.com',
    description='EKF2 visual GPS + IMU fusion node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf2_fusion_node = ekf2_fusion_pkg.ekf2_fusion_node:main',
        ],
    },
)
