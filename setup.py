import os
from glob import glob

from setuptools import setup

package_name = 'quest_webxr_bridge'


def tree(src, dst):
    """data_files entries that install every file under src into dst, keeping subdirectories."""
    entries = []
    for dirpath, _, filenames in os.walk(src):
        files = [os.path.join(dirpath, f) for f in filenames]
        if files:
            entries.append((os.path.join(dst, os.path.relpath(dirpath, src)), files))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ] + tree('web', 'share/' + package_name + '/web'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szmlb',
    maintainer_email='6672969+szmlb@users.noreply.github.com',
    description='Meta Quest WebXR to ROS2 bridge producing Quest2ROS2-compatible topics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webxr_bridge = quest_webxr_bridge.webxr_bridge:main',
            'fake_webxr_publisher = quest_webxr_bridge.fake_webxr_publisher:main',
        ],
    },
)
