"""
PlatformIO pre-build script: work around a kilted-only bug in
micro_ros_platformio's dev-environment build.

The issue
---------
`kilted` branch of `ros2/ament_cmake_ros` ships a new `rmw_test_fixture`
package that find_package()s `rmw` — but `rmw` is in micro_ros_platformio's
mcu_environments dict, not dev_environments, so the dev colcon build fails
with `CMake did not find ... rmw`. Upstream PR
https://github.com/micro-ROS/micro_ros_platformio/pull/189 fixes it by
adding `--packages-ignore rmw_test_fixture` to the dev colcon invocation;
unmerged as of 2026-06.

The patch
---------
We rewrite the colcon command line inside the installed copy of
`microros_utils/library_builder.py` before the library's own extra_script
imports it. Idempotent — re-runs no-op once the marker token is present.

Drop this script (and its pre: hook in platformio.ini) once the upstream
PR merges and we bump to a release tag that includes it.
"""

Import("env")

import os
import re


def _patch_library_builder():
    libdeps_dir = env.get("PROJECT_LIBDEPS_DIR")
    pioenv = env.get("PIOENV")
    if not libdeps_dir or not pioenv:
        return

    target = os.path.join(
        libdeps_dir, pioenv, "micro_ros_platformio",
        "microros_utils", "library_builder.py",
    )
    if not os.path.isfile(target):
        return

    with open(target, "r") as f:
        content = f.read()

    marker = "--packages-ignore rmw_test_fixture"
    if marker in content:
        return

    needle = "colcon build --cmake-args"
    replacement = "colcon build --packages-ignore rmw_test_fixture --cmake-args"
    patched, n = re.subn(re.escape(needle), replacement, content, count=1)
    if n != 1:
        print("patch_microros: WARNING — colcon line not found in {}".format(target))
        return

    with open(target, "w") as f:
        f.write(patched)
    print("patch_microros: applied rmw_test_fixture skip to {}".format(target))


_patch_library_builder()
