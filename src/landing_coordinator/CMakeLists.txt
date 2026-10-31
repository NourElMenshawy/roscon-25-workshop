cmake_minimum_required(VERSION 3.20)
project(landing_coordinator)

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
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# ============================================================
# Landing request service
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestLanding.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# ============================================================
# Landing arbitration library
# ============================================================

add_library(landing_arbitrator
  src/landing_arbitrator.cpp
)

target_include_directories(landing_arbitrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================
# Landing coordinator node library
# ============================================================

add_library(landing_coordinator_node_lib
  src/landing_coordinator.cpp
)

target_include_directories(landing_coordinator_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(landing_coordinator_node_lib PUBLIC
  rclcpp::rclcpp
  landing_arbitrator
  ${cpp_typesupport_target}
)

# ============================================================
# Landing coordinator executable
# ============================================================

add_executable(landing_coordinator_node
  src/main.cpp
)

target_link_libraries(landing_coordinator_node PRIVATE
  rclcpp::rclcpp
  landing_coordinator_node_lib
)

# ============================================================
# Mock request client executable
# ============================================================

add_executable(mock_request_client
  src/mock_request_client.cpp
)

target_link_libraries(mock_request_client PRIVATE
  rclcpp::rclcpp
  ${cpp_typesupport_target}
  Threads::Threads
)

# ============================================================
# Installation
# ============================================================

install(TARGETS
  landing_arbitrator
  landing_coordinator_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  landing_coordinator_node
  mock_request_client
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

  ament_add_gtest(test_landing_arbitrator
    test/test_landing_arbitrator.cpp
  )

  if(TARGET test_landing_arbitrator)
    target_link_libraries(test_landing_arbitrator
      landing_arbitrator
      Threads::Threads
    )
  endif()

  ament_add_gtest(test_landing_service
    test/test_landing_service.cpp
    TIMEOUT 45
  )

  if(TARGET test_landing_service)
    target_link_libraries(test_landing_service
      rclcpp::rclcpp
      landing_coordinator_node_lib
      ${cpp_typesupport_target}
      Threads::Threads
    )
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
