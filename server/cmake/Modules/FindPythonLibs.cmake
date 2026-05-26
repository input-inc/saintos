# Compatibility shim for the deprecated FindPythonLibs module.
#
# CMake 3.27 removed FindPythonLibs.cmake. ROS2 Humble's python_cmake_module
# still calls `find_package(PythonLibs ...)`, so on CMake 3.27+ that call fails
# unless this shim is on CMAKE_MODULE_PATH.
#
# Delegates to the modern FindPython3 module and maps the results back to the
# legacy PYTHONLIBS_* / PYTHON_* variable names that callers expect.
#
# We request Interpreter + Development so the chosen interpreter and library
# always match — this is the bug the previous shim had: asking for Development
# alone could pick a different Python install than the prior FindPythonInterp
# call, and on linux/arm64 FindPython3 would silently fail with no diagnostic.

set(_findpythonlibs_args)
if(PythonLibs_FIND_VERSION)
  list(APPEND _findpythonlibs_args ${PythonLibs_FIND_VERSION})
endif()
if(PythonLibs_FIND_REQUIRED)
  list(APPEND _findpythonlibs_args REQUIRED)
endif()
if(PythonLibs_FIND_QUIETLY)
  list(APPEND _findpythonlibs_args QUIET)
endif()

find_package(Python3 ${_findpythonlibs_args} COMPONENTS Interpreter Development)
unset(_findpythonlibs_args)

set(PYTHONLIBS_FOUND ${Python3_Development_FOUND})
set(PYTHONLIBS_VERSION_STRING "${Python3_VERSION}")
set(PYTHON_LIBRARIES "${Python3_LIBRARIES}")
set(PYTHON_LIBRARY "${Python3_LIBRARIES}")
set(PYTHON_INCLUDE_DIRS "${Python3_INCLUDE_DIRS}")
set(PYTHON_INCLUDE_DIR "${Python3_INCLUDE_DIRS}")
set(PYTHON_DEBUG_LIBRARIES "${Python3_LIBRARIES}")
