# FindPythonInterp.cmake - Compatibility shim for CMake 3.27+
#
# CMake 3.27 removed FindPythonInterp. This shim provides backwards compatibility
# by using FindPython3 and setting the legacy variables.
#
# This module sets the following variables:
#   PYTHONINTERP_FOUND - Was the Python interpreter found
#   PYTHON_EXECUTABLE - Path to the Python interpreter
#   PYTHON_VERSION_STRING - Python version found e.g. 3.11.5
#   PYTHON_VERSION_MAJOR - Python major version found e.g. 3
#   PYTHON_VERSION_MINOR - Python minor version found e.g. 11
#   PYTHON_VERSION_PATCH - Python patch version found e.g. 5

if(NOT Python3_FOUND)
  find_package(Python3 COMPONENTS Interpreter)
endif()

if(Python3_Interpreter_FOUND OR Python3_FOUND)
  set(PYTHONINTERP_FOUND TRUE)
  set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
  set(PYTHON_VERSION_STRING ${Python3_VERSION})
  set(PYTHON_VERSION_MAJOR ${Python3_VERSION_MAJOR})
  set(PYTHON_VERSION_MINOR ${Python3_VERSION_MINOR})
  set(PYTHON_VERSION_PATCH ${Python3_VERSION_PATCH})

  # Handle REQUIRED argument
  if(PythonInterp_FIND_REQUIRED)
    # Already found, nothing to do
  endif()

  # Handle version check
  if(PythonInterp_FIND_VERSION)
    if(PYTHON_VERSION_STRING VERSION_LESS PythonInterp_FIND_VERSION)
      if(PythonInterp_FIND_REQUIRED)
        message(FATAL_ERROR "Python ${PythonInterp_FIND_VERSION} required, found ${PYTHON_VERSION_STRING}")
      else()
        set(PYTHONINTERP_FOUND FALSE)
      endif()
    endif()
  endif()
else()
  set(PYTHONINTERP_FOUND FALSE)
  if(PythonInterp_FIND_REQUIRED)
    message(FATAL_ERROR "Python interpreter not found")
  endif()
endif()

# Provide result variables for find_package_handle_standard_args compatibility
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PythonInterp
  REQUIRED_VARS PYTHON_EXECUTABLE
  VERSION_VAR PYTHON_VERSION_STRING
)
