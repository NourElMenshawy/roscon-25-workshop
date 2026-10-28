cmake_minimum_required(VERSION 3.20)

project(px4_tf)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  px4_tf_publisher
  src/px4_tf_publisher.cpp
  src/px4_tf_publisher_node.cpp
)

target_include_directories(px4_tf_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(
  px4_tf_publisher
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  px4_tf_publisher
  PRIVATE
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${px4_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)

install(
  TARGETS
    px4_tf_publisher
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
