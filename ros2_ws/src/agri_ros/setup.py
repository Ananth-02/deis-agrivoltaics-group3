from setuptools import setup

package_name = 'agri_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DEIS Group 3',
    maintainer_email='112279457+Ananth-02@users.noreply.github.com',
    description='ROS2 wrappers for the agri_loop gateway and panel simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gateway_node = agri_ros.gateway_node:main',
            'panel_sim_node = agri_ros.panel_sim_node:main',
        ],
    },
)
