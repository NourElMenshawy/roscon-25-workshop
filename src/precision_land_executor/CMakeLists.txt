cmake_minimum_required(VERSION 3.20)
project(precision_land_executor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
# ============================================================
# stdexec interface library
# ============================================================

set(STDEXEC_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/stdexec/include"
)

if(NOT EXISTS "${STDEXEC_INCLUDE_DIR}/stdexec/execution.hpp")
  message(FATAL_ERROR
    "stdexec headers not found at ${STDEXEC_INCLUDE_DIR}"
  )
endif()

add_library(stdexec_headers INTERFACE)

target_include_directories(stdexec_headers
  SYSTEM INTERFACE
    "${STDEXEC_INCLUDE_DIR}"
)

target_link_libraries(stdexec_headers INTERFACE
  Threads::Threads
  TBB::tbb
)

target_compile_features(stdexec_headers
  INTERFACE cxx_std_20
)

# ============================================================
# PX4 Precision Landing executable
# ============================================================

add_executable(${PROJECT_NAME}
  CustomMode_v2.cpp
  CustomModeExecutor_v2.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  rclcpp::rclcpp
  px4_ros2_cpp::px4_ros2_cpp
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${px4_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
  stdexec_headers
)


# ============================================================
# stdexec smoke test executable
# ============================================================

add_executable(stdexec_smoke
  stdexec_smoke.cpp
)

target_link_libraries(stdexec_smoke PRIVATE
  stdexec_headers
)

# ============================================================
# Installation
# ============================================================

install(TARGETS
  ${PROJECT_NAME}
  stdexec_smoke
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
