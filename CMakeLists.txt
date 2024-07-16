cmake_minimum_required(VERSION 3.8.2)
project(qrb_ros_camera
        LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

include_directories (./include)

include_directories (${SYSROOT_INCDIR}/qmmf-sdk/common/utils PRIVATE)
include_directories (${SYSROOT_INCDIR}/qmmf-sdk/include PRIVATE)
include_directories (${SYSROOT_INCDIR}/qmmf-sdk PRIVATE)
include_directories (${SYSROOT_INCDIR}/opencv4)
include_directories (${SYSROOT_INCDIR}/camx-api/camx/service)
include_directories (${SYSROOT_INCDIR}/glib-2.0)
include_directories (${SYSROOT_LIBDIR}/glib-2.0/include)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(qrb_ros_camera src/main.cpp)

ament_auto_add_library(qrb_camera SHARED
  src/camera_ros2_config.cpp
  src/camera_ros2_node.cpp
  src/qmmf_camera_parameters.cpp
  src/qmmf_camera.cpp
  src/qmmf_ros2_pipeline.cpp)

rclcpp_components_register_nodes(qrb_camera "qrb_ros::camera::CameraNode")

target_link_libraries( qrb_camera
  camera_metadata
  qmmf_recorder_client
  glib-2.0
  qmmf_utils
  yaml-cpp
)

target_link_libraries(qrb_ros_camera
  qrb_camera
)

ament_auto_package(INSTALL_TO_SHARE config launch)
















