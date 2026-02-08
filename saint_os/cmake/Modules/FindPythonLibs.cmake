# FindPythonLibs.cmake - Compatibility shim for CMake 3.27+
#
# CMake 3.27 removed FindPythonLibs. This shim provides backwards compatibility
# by using FindPython3 and setting the legacy variables.
#
# This module sets the following variables:
#   PYTHONLIBS_FOUND - Was the Python libraries found
#   PYTHON_LIBRARIES - Path to the Python library
#   PYTHON_INCLUDE_DIRS - Path to Python include directories
#   PYTHON_LIBRARY - Path to the Python library (singular)
#   PYTHON_INCLUDE_DIR - Path to Python include directory (singular)

if(NOT Python3_FOUND)
  find_package(Python3 COMPONENTS Development)
endif()

if(Python3_Development_FOUND OR (Python3_FOUND AND Python3_LIBRARIES))
  set(PYTHONLIBS_FOUND TRUE)
  set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
  set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
  set(PYTHON_LIBRARY ${Python3_LIBRARIES})
  set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS})
  set(PYTHON_DEBUG_LIBRARIES ${Python3_LIBRARIES})

  # Handle version check
  if(PythonLibs_FIND_VERSION)
    if(Python3_VERSION VERSION_LESS PythonLibs_FIND_VERSION)
      if(PythonLibs_FIND_REQUIRED)
        message(FATAL_ERROR "Python ${PythonLibs_FIND_VERSION} required, found ${Python3_VERSION}")
      else()
        set(PYTHONLIBS_FOUND FALSE)
      endif()
    endif()
  endif()
else()
  set(PYTHONLIBS_FOUND FALSE)
  if(PythonLibs_FIND_REQUIRED)
    message(FATAL_ERROR "Python libraries not found")
  endif()
endif()

# Provide result variables for find_package_handle_standard_args compatibility
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PythonLibs
  REQUIRED_VARS PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS
  VERSION_VAR Python3_VERSION
)
