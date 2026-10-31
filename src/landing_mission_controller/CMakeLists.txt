cmake_minimum_required(VERSION 3.20)
project(landing_mission_controller)

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
find_package(geometry_msgs REQUIRED)
find_package(landing_coordinator REQUIRED)

# ============================================================
# Mission policy (ROS-independent)
# ============================================================

add_library(landing_mission_policy
  src/mission_policy.cpp
)

target_include_directories(landing_mission_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================
# Mission controller node library
# ============================================================

add_library(landing_mission_controller_lib
  src/landing_mission_controller.cpp
)

target_include_directories(landing_mission_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(landing_mission_controller_lib PUBLIC
  landing_mission_policy
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${landing_coordinator_TARGETS}
)

# ============================================================
# Mission controller executable
# ============================================================

add_executable(landing_mission_controller_node
  src/main.cpp
)

target_link_libraries(landing_mission_controller_node PRIVATE
  landing_mission_controller_lib
)

# ============================================================
# Mock ArUco observation publisher (simulation-only)
# ============================================================

add_executable(mock_target_publisher
  src/mock_target_publisher.cpp
)

target_link_libraries(mock_target_publisher PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

# ============================================================
# Installation
# ============================================================

install(TARGETS
  landing_mission_policy
  landing_mission_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  landing_mission_controller_node
  mock_target_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_policy
    test/test_mission_policy.cpp
  )
  if(TARGET test_mission_policy)
    # ament_add_gtest uses the plain linking signature internally.
    target_link_libraries(test_mission_policy
      landing_mission_policy
    )
  endif()

  ament_add_gtest(test_controller_integration
    test/test_controller_integration.cpp
    TIMEOUT 40
  )
  if(TARGET test_controller_integration)
    target_link_libraries(test_controller_integration
      landing_mission_controller_lib
    )
  endif()
endif()

ament_package()
