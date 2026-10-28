cmake_minimum_required(VERSION 3.20)

project(aruco_tracker)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)

find_package(
  OpenCV REQUIRED
  COMPONENTS
    core
    imgproc
    calib3d
    aruco
)

add_executable(
  aruco_tracker
  src/ArucoTracker.cpp
)

target_compile_features(
  aruco_tracker
  PRIVATE
    cxx_std_20
)

target_include_directories(
  aruco_tracker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  aruco_tracker
  PRIVATE
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${cv_bridge_TARGETS}
    tf2::tf2
    ${OpenCV_LIBS}
)

install(
  TARGETS
    aruco_tracker
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    cfg
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
