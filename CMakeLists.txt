cmake_minimum_required(VERSION 3.8)
project(volleyball_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_core SHARED
  src/aero_ekf.cpp
  src/trajectory_predictor.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

add_executable(volleyball_tracker_node
  src/tracker_node.cpp
)

target_link_libraries(volleyball_tracker_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(volleyball_tracker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS
  ${PROJECT_NAME}_core
  volleyball_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
