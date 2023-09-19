cmake_minimum_required(VERSION 3.5)
project(odometry_estimator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)



# Vehicle models lib
add_library(vehicle_models src/vehicle_models/vehicle_models.cpp)
target_include_directories(
  vehicle_models
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/vehicle_models>
         $<INSTALL_INTERFACE:include>)

# Node
add_executable(odometry_estimator src/node/odometry_estimation_node.cpp)
ament_target_dependencies(odometry_estimator rclcpp nav_msgs std_msgs vesc_msgs tf2_geometry_msgs)

add_executable(erpm_to_rpm src/node/erpm_to_rpm.cpp)
ament_target_dependencies(erpm_to_rpm rclcpp vesc_msgs std_msgs )

add_executable(avg_erpm_publisher src/node/avg_erpm_publisher.cpp)
ament_target_dependencies(avg_erpm_publisher rclcpp vesc_msgs )

add_executable(odometry_estimation_new src/node/odometry_estimation_new.cpp)
ament_target_dependencies(odometry_estimation_new rclcpp nav_msgs vesc_msgs tf2_geometry_msgs geometry_msgs tf2_ros)

add_executable(odometry_with_displacement src/node/odometry_with_displacement.cpp)
ament_target_dependencies(odometry_with_displacement rclcpp nav_msgs vesc_msgs tf2_geometry_msgs geometry_msgs tf2_ros)

add_executable(revolution_publisher src/node/revolution_publisher.cpp)
ament_target_dependencies(revolution_publisher rclcpp std_msgs vesc_msgs)


target_link_libraries(odometry_estimator vehicle_models)
target_include_directories(
  odometry_estimator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/node>
         $<INSTALL_INTERFACE:include>)


install(TARGETS 
  odometry_estimator 
  erpm_to_rpm
  avg_erpm_publisher
  odometry_estimation_new
  odometry_with_displacement
  revolution_publisher
  
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
