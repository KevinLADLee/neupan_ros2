from setuptools import setup
import glob
import os

package_name = 'neupan_ros2'

launch_files = glob.glob('launch/*.py')
rviz_files = glob.glob('rviz/*.rviz')

# Recursively collect all config files while preserving directory structure
def get_data_files(directory):
    data_files = []
    for root, dirs, files in os.walk(directory):
        if files:
            # Get relative path from the source directory
            rel_dir = os.path.relpath(root, '.')
            # Create installation path
            install_dir = os.path.join('share', package_name, rel_dir)
            # Get full paths of all files in this directory
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_paths))
    return data_files

config_data_files = get_data_files('config')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/rviz', rviz_files)
    ] + config_data_files,
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
