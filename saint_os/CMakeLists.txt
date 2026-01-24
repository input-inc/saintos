cmake_minimum_required(VERSION 3.8)
project(saint_os)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# Message and Service Generation
# ==============================================================================

set(MSG_FILES
  "msg/NodeGPIOStatus.msg"
  "msg/GPIOPin.msg"
  "msg/PeripheralInfo.msg"
  "msg/NodeAnnouncement.msg"
  "msg/LiveLinkFrame.msg"
  "msg/LiveLinkSubject.msg"
  "msg/RCReceiverState.msg"
  "msg/RCChannelState.msg"
  "msg/RouteStatus.msg"
  "msg/HeadState.msg"
  "msg/HeadCommand.msg"
  "msg/ArmState.msg"
  "msg/ArmCommand.msg"
  "msg/GripperCommand.msg"
  "msg/GripperState.msg"
  "msg/TrackState.msg"
  "msg/ConsoleInput.msg"
  "msg/SystemStatus.msg"
)

set(SRV_FILES
  "srv/FirmwareCheck.srv"
  "srv/FirmwareDownload.srv"
  "srv/AdoptNode.srv"
  "srv/ResetNode.srv"
  "srv/GPIOProbe.srv"
  "srv/SystemCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces
)

# ==============================================================================
# Python Package Installation
# ==============================================================================

# Install Python packages
ament_python_install_package(saint_server)
ament_python_install_package(saint_common)

# ==============================================================================
# Install Launch Files
# ==============================================================================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Install Configuration Files
# ==============================================================================

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Install Web Interface
# ==============================================================================

install(DIRECTORY
  web
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Install Scripts
# ==============================================================================

install(PROGRAMS
  scripts/build_firmware.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Testing
# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Exclude specific linters if needed
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # Python tests
  ament_add_pytest_test(test_server test/test_server.py)
  ament_add_pytest_test(test_router test/test_router.py)
  ament_add_pytest_test(test_websocket test/test_websocket.py)
endif()

# ==============================================================================
# Export and Finalize
# ==============================================================================

ament_export_dependencies(
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosidl_default_runtime
)

ament_package()
