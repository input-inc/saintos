"""
SAINT.OS Server Package

Main server components for the SAINT.OS robot control system.
"""

__version__ = '0.5.0'
__author__ = 'SAINT.OS Team'

from saint_server.server_node import SaintServerNode

__all__ = [
    'SaintServerNode',
    '__version__',
]
