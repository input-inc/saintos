"""Setup script for SAINT.OS Python packages."""

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'saint_os'


def get_data_files():
    """Collect all data files for installation."""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    # Add config files
    config_dir = os.path.join(os.path.dirname(__file__), 'config')
    if os.path.isdir(config_dir):
        # Main config files
        config_files = glob(os.path.join(config_dir, '*.yaml'))
        if config_files:
            data_files.append((f'share/{package_name}/config', config_files))

        # Robot manifests (platform identity + node role names)
        robots_dir = os.path.join(config_dir, 'robots')
        if os.path.isdir(robots_dir):
            robot_files = glob(os.path.join(robots_dir, '*.yaml'))
            if robot_files:
                data_files.append((f'share/{package_name}/config/robots', robot_files))

    # Vite build output — the production UI. Walk recursively to
    # preserve the assets/ subdir Vite emits. Built on the host (or
    # CI) before colcon runs, see web/MIGRATION.md.
    web_dir = os.path.join(os.path.dirname(__file__), 'web')
    dist_dir = os.path.join(web_dir, 'dist')
    if os.path.isdir(dist_dir):
        for root, _dirs, files in os.walk(dist_dir):
            if not files:
                continue
            rel = os.path.relpath(root, web_dir)             # "dist" or "dist/assets"
            bucket = f'share/{package_name}/web/{rel}'
            data_files.append((bucket, [os.path.join(root, f) for f in files]))

    return data_files


setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=[
        'setuptools',
        'websockets>=10.0',
        'aiohttp>=3.8.0',
        'pyyaml>=6.0',
        'numpy>=1.21.0',
        'psutil>=5.9.0',
        # bleak powers the host_controller's in-process BLE BMS
        # driver (saint_server/host_peripherals/). On Linux/Pi it
        # talks to BlueZ over D-Bus; the `saint` service user must be
        # in the `bluetooth` group. Import is lazy in the driver, so
        # a host without bleak still loads the server — only the
        # host_controller BLE features fail to come up.
        #
        # Floor is 0.20 because that's what Debian Bookworm ships as
        # `python3-bleak`. APIs used by the driver
        # (BleakClient.disconnected_callback, BleakScanner.start/stop,
        # advert_data.rssi) all landed by 0.20.
        'bleak>=0.20',
    ],
    extras_require={
        'dev': [
            'pytest>=7.0.0',
            'pytest-asyncio>=0.20.0',
            'pytest-cov>=4.0.0',
            'black>=23.0.0',
            'isort>=5.12.0',
            'mypy>=1.0.0',
            'flake8>=6.0.0',
        ],
        'pi': [
            'RPi.GPIO>=0.7.0',
            'pigpio>=1.78',
            'smbus2>=0.4.0',
            'spidev>=3.5',
        ],
    },
    zip_safe=True,
    maintainer='SAINT.OS Team',
    maintainer_email='maintainer@example.com',
    description='SAINT.OS - System for Articulated Intelligence and Navigation Tasks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'saint_server = saint_server.server_node:main',
            'saint_webserver = saint_server.webserver:main',
        ],
    },
    python_requires='>=3.10',
)
