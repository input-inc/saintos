"""Setup script for SAINT.OS Python packages."""

from setuptools import setup, find_packages

package_name = 'saint_os'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'websockets>=10.0',
        'aiohttp>=3.8.0',
        'pyyaml>=6.0',
        'numpy>=1.21.0',
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
