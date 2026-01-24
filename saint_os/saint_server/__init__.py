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

# Lazy imports for optional components
def __getattr__(name):
    if name == 'webserver':
        from saint_server import webserver
        return webserver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
