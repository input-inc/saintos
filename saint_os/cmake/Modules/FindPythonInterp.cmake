# Compatibility shim for the deprecated FindPythonInterp module.
#
# CMake 3.27 removed FindPythonInterp.cmake. ROS2 Humble's python_cmake_module
# still calls `find_package(PythonInterp ...)`, so on CMake 3.27+ that call fails
# unless this shim is on CMAKE_MODULE_PATH.
#
# Delegates to the modern FindPython3 module and maps the results back to the
# legacy PYTHON_* / PYTHONINTERP_* variable names that callers expect.

set(_findpythoninterp_args)
if(PythonInterp_FIND_VERSION)
  list(APPEND _findpythoninterp_args ${PythonInterp_FIND_VERSION})
endif()
if(PythonInterp_FIND_REQUIRED)
  list(APPEND _findpythoninterp_args REQUIRED)
endif()
if(PythonInterp_FIND_QUIETLY)
  list(APPEND _findpythoninterp_args QUIET)
endif()

find_package(Python3 ${_findpythoninterp_args} COMPONENTS Interpreter)
unset(_findpythoninterp_args)

set(PYTHONINTERP_FOUND ${Python3_Interpreter_FOUND})
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
set(PYTHON_VERSION_STRING "${Python3_VERSION}")
set(PYTHON_VERSION_MAJOR "${Python3_VERSION_MAJOR}")
set(PYTHON_VERSION_MINOR "${Python3_VERSION_MINOR}")
set(PYTHON_VERSION_PATCH "${Python3_VERSION_PATCH}")
