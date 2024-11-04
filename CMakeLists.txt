cmake_minimum_required(VERSION 3.8)
project(r4f_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(r4f_msgs REQUIRED)

find_package(realsense2 REQUIRED)
find_package(OpenCV 4.10 REQUIRED)

find_package(cv_bridge REQUIRED)

add_subdirectory(include/r4f_sensors)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RGBD.msg"
  "msg/GnssReading.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Include directories
include_directories(
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
  ${realsense2_INCLUDE_DIRS}
)

add_executable(realsense_sim src/realsense_sim.cpp)
ament_target_dependencies(realsense_sim
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  image_transport
  rosidl_typesupport_cpp
  realsense2
  r4f_msgs
)

add_executable(gnss_sim src/gnss_sim.cpp)
ament_target_dependencies(gnss_sim
  rclcpp
  std_msgs
  sensor_msgs
  rosidl_typesupport_cpp
)

# Get the message type support target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Link additional libraries explicitly if necessary
target_link_libraries(realsense_sim
  ${cpp_typesupport_target}
  ${realsense2_LIBRARY}
  ${OpenCV_LIBS}
)

target_link_libraries(gnss_sim
  ${cpp_typesupport_target}
  gnss_compass_utils
)

ament_export_dependencies(rosidl_default_runtime)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  realsense_sim
  gnss_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
