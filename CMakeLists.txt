cmake_minimum_required(VERSION 3.8)
project(r4f_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories
include_directories(
  include
  ${realsense2_INCLUDE_DIRS}  # Include RealSense2 headers
)

add_executable(realsense_sim src/realsense_sim.cpp)
ament_target_dependencies(realsense_sim 
  rclcpp 
  std_msgs 
  realsense2_camera_msgs 
  realsense2
  )

  # Link additional libraries explicitly if necessary
target_link_libraries(realsense_sim 
  ${realsense2_LIBRARY})


install(TARGETS
  realsense_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
