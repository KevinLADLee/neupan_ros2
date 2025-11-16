from setuptools import setup
import glob

package_name = 'neupan_ros2'

launch_files = glob.glob('launch/*.py')
config_files = glob.glob('config/*.yaml')
neupan_config_files = glob.glob('config/neupan_config/*')
dune_checkpoint_files = glob.glob('config/dune_checkpoint/*')
rviz_files = glob.glob('rviz/*.rviz')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/config/neupan_config', neupan_config_files),
        ('share/' + package_name + '/config/dune_checkpoint', dune_checkpoint_files),
        ('share/' + package_name + '/rviz', rviz_files)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevinlad',
    maintainer_email='kevinladlee@gmail.com',
    description=(
        'NeuPAN: Neural Proximal Alternating Network for '
        'autonomous robot navigation with ROS2 integration'
    ),
    license='GPL-3.0',
    # tests_require=['pytest'],
    keywords=['ROS'],
    entry_points={
        'console_scripts': [
            'neupan_node = neupan_ros2.neupan_node:main'
        ],
    },
)
