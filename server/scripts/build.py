#!/usr/bin/env python3
"""
Cross-platform build script for SAINT.OS

This script handles building the SAINT.OS ROS2 package on macOS, Linux, and Windows.
"""

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


class BuildError(Exception):
    """Build error exception."""
    pass


def get_platform():
    """Get the current platform."""
    system = platform.system().lower()
    if system == 'darwin':
        return 'macos'
    elif system == 'linux':
        return 'linux'
    elif system == 'windows':
        return 'windows'
    else:
        raise BuildError(f'Unsupported platform: {system}')


def find_ros2_setup():
    """Find the ROS2 setup script."""
    plat = get_platform()

    # Check if ROS2 is already available (e.g., via Conda/RoboStack)
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro and shutil.which('ros2'):
        return 'ALREADY_SOURCED'

    # Check Conda environment (RoboStack)
    conda_prefix = os.environ.get('CONDA_PREFIX')
    if conda_prefix:
        conda_setup = os.path.join(conda_prefix, 'setup.bash' if plat != 'windows' else 'setup.bat')
        if os.path.exists(conda_setup):
            return conda_setup
        # RoboStack may have ros2 available without setup script
        if shutil.which('ros2'):
            return 'ALREADY_SOURCED'

    # Common ROS2 installation paths
    ros2_paths = {
        'macos': [
            '/opt/ros/jazzy/setup.bash',
            '/opt/ros/iron/setup.bash',
            '/opt/ros/humble/setup.bash',
            os.path.expanduser('~/ros2_humble/install/setup.bash'),
            os.path.expanduser('~/ros2_humble/setup.bash'),
        ],
        'linux': [
            '/opt/ros/jazzy/setup.bash',
            '/opt/ros/iron/setup.bash',
            '/opt/ros/humble/setup.bash',
        ],
        'windows': [
            'C:/opt/ros/jazzy/setup.bat',
            'C:/opt/ros/iron/setup.bat',
            'C:/opt/ros/humble/setup.bat',
            'C:/dev/ros2_humble/setup.bat',
        ],
    }

    for path in ros2_paths.get(plat, []):
        if os.path.exists(path):
            return path

    # Check ROS_DISTRO environment variable for custom install paths
    if ros_distro:
        if plat == 'windows':
            path = f'C:/opt/ros/{ros_distro}/setup.bat'
        else:
            path = f'/opt/ros/{ros_distro}/setup.bash'
        if os.path.exists(path):
            return path

    return None


def run_command(cmd, cwd=None, env=None, shell=False):
    """Run a command and return the result."""
    print(f'Running: {" ".join(cmd) if isinstance(cmd, list) else cmd}')

    try:
        result = subprocess.run(
            cmd,
            cwd=cwd,
            env=env,
            shell=shell,
            check=True,
            capture_output=False,
        )
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f'Command failed with exit code {e.returncode}')
        return False


def source_ros2_and_run(ros2_setup, cmd, cwd=None):
    """Source ROS2 and run a command."""
    plat = get_platform()

    # If ROS2 is already sourced (e.g., via Conda), run command directly
    if ros2_setup == 'ALREADY_SOURCED':
        if plat == 'windows':
            return run_command(cmd, cwd=cwd, shell=True)
        else:
            return run_command(['bash', '-c', cmd], cwd=cwd)

    if plat == 'windows':
        # Windows: use call to source batch file
        full_cmd = f'call "{ros2_setup}" && {cmd}'
        return run_command(full_cmd, cwd=cwd, shell=True)
    else:
        # Unix: source setup.bash
        full_cmd = f'source "{ros2_setup}" && {cmd}'
        return run_command(['bash', '-c', full_cmd], cwd=cwd)


def check_dependencies():
    """Check that required dependencies are installed."""
    print('Checking dependencies...')

    # Check Python version
    if sys.version_info < (3, 10):
        raise BuildError(f'Python 3.10+ required, found {sys.version}')
    print(f'  Python: {sys.version}')

    # Check Conda environment
    conda_env = os.environ.get('CONDA_DEFAULT_ENV')
    if conda_env:
        print(f'  Conda env: {conda_env}')

    # Check ROS2
    ros2_setup = find_ros2_setup()
    if not ros2_setup:
        raise BuildError(
            'ROS2 installation not found.\n\n'
            'Please install ROS2 using one of these methods:\n'
            '  macOS:  Use RoboStack with Conda (see INSTALL.md)\n'
            '  Linux:  sudo apt install ros-humble-desktop\n'
            '  All:    Build from source\n\n'
            'If using Conda/RoboStack, activate the environment first:\n'
            '  conda activate ros2_env'
        )

    if ros2_setup == 'ALREADY_SOURCED':
        ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
        print(f'  ROS2: already sourced (ROS_DISTRO={ros_distro})')
    else:
        print(f'  ROS2 setup: {ros2_setup}')

    # Check colcon
    plat = get_platform()
    if plat == 'windows':
        colcon_check = 'where colcon'
    else:
        colcon_check = 'which colcon'

    if not source_ros2_and_run(ros2_setup, colcon_check):
        raise BuildError('colcon not found. Install with: pip install colcon-common-extensions')
    print('  colcon: found')

    return ros2_setup


def clean_build(workspace_path):
    """Clean build artifacts."""
    print('Cleaning build artifacts...')

    dirs_to_remove = ['build', 'install', 'log']
    for dir_name in dirs_to_remove:
        dir_path = workspace_path / dir_name
        if dir_path.exists():
            print(f'  Removing {dir_path}')
            shutil.rmtree(dir_path)

    print('Clean complete.')


def build_package(ros2_setup, workspace_path, build_type='Release', parallel_jobs=None):
    """Build the SAINT.OS package."""
    print(f'Building SAINT.OS ({build_type})...')

    # Construct colcon build command
    cmd_parts = [
        'colcon', 'build',
        '--symlink-install',
        f'--cmake-args', f'-DCMAKE_BUILD_TYPE={build_type}',
    ]

    if parallel_jobs:
        cmd_parts.extend(['--parallel-workers', str(parallel_jobs)])

    cmd = ' '.join(cmd_parts)

    if not source_ros2_and_run(ros2_setup, cmd, cwd=str(workspace_path)):
        raise BuildError('Build failed')

    print('Build complete.')


def run_tests(ros2_setup, workspace_path):
    """Run the test suite."""
    print('Running tests...')

    cmd = 'colcon test --pytest-args -v'

    if not source_ros2_and_run(ros2_setup, cmd, cwd=str(workspace_path)):
        print('Warning: Some tests failed')
        return False

    # Show test results
    source_ros2_and_run(ros2_setup, 'colcon test-result --verbose', cwd=str(workspace_path))

    print('Tests complete.')
    return True


def install_python_deps(dev=False):
    """Install Python dependencies."""
    print('Installing Python dependencies...')

    cmd = [sys.executable, '-m', 'pip', 'install', '-e', '.']
    if dev:
        cmd[-1] = '.[dev]'

    package_path = Path(__file__).parent.parent

    if not run_command(cmd, cwd=str(package_path)):
        raise BuildError('Failed to install Python dependencies')

    print('Python dependencies installed.')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Build SAINT.OS')
    parser.add_argument('--clean', action='store_true', help='Clean build artifacts before building')
    parser.add_argument('--release', action='store_true', help='Build in Release mode (default)')
    parser.add_argument('--debug', action='store_true', help='Build in Debug mode')
    parser.add_argument('--test', action='store_true', help='Run tests after building')
    parser.add_argument('--jobs', '-j', type=int, help='Number of parallel jobs')
    parser.add_argument('--install-deps', action='store_true', help='Install Python dependencies')
    parser.add_argument('--dev', action='store_true', help='Install development dependencies')

    args = parser.parse_args()

    # Determine workspace path (parent of saint_os package)
    script_path = Path(__file__).resolve()
    package_path = script_path.parent.parent
    workspace_path = package_path.parent

    print(f'SAINT.OS Build Script')
    print(f'Platform: {get_platform()}')
    print(f'Workspace: {workspace_path}')
    print()

    try:
        # Check dependencies
        ros2_setup = check_dependencies()
        print()

        # Install Python deps if requested
        if args.install_deps:
            install_python_deps(dev=args.dev)
            print()

        # Clean if requested
        if args.clean:
            clean_build(workspace_path)
            print()

        # Determine build type
        build_type = 'Debug' if args.debug else 'Release'

        # Build
        build_package(ros2_setup, workspace_path, build_type, args.jobs)
        print()

        # Test if requested
        if args.test:
            run_tests(ros2_setup, workspace_path)
            print()

        print('=' * 60)
        print('Build successful!')
        print()
        print('To use the built package, source the setup script:')
        if get_platform() == 'windows':
            print(f'  call {workspace_path}\\install\\setup.bat')
        else:
            print(f'  source {workspace_path}/install/setup.bash')
        print()

    except BuildError as e:
        print(f'\nBuild error: {e}', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print('\nBuild cancelled.')
        sys.exit(1)


if __name__ == '__main__':
    main()
