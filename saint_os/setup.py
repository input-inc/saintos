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

        # Role definitions
        roles_dir = os.path.join(config_dir, 'roles')
        if os.path.isdir(roles_dir):
            role_files = glob(os.path.join(roles_dir, '*.yaml'))
            if role_files:
                data_files.append((f'share/{package_name}/config/roles', role_files))

    # Add web files
    web_dir = os.path.join(os.path.dirname(__file__), 'web')
    if os.path.isdir(web_dir):
        # HTML files
        html_files = glob(os.path.join(web_dir, '*.html'))
        if html_files:
            data_files.append((f'share/{package_name}/web', html_files))

        # JavaScript files
        js_dir = os.path.join(web_dir, 'js')
        if os.path.isdir(js_dir):
            js_files = glob(os.path.join(js_dir, '*.js'))
            if js_files:
                data_files.append((f'share/{package_name}/web/js', js_files))

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
