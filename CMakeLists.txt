cmake_minimum_required(VERSION 3.10)
project(ruitong_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas -Wno-endif-labels)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToolTrackingMsg.msg"
  "msg/TrackingDataMsg.msg"
  DEPENDENCIES std_msgs
)

# Set RUITONG SDK library path
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(LIB_NAME ARMDsterotracking)

# ROS2 node executable
add_executable(ruitong_tracking_node src/ruitong_tracking_node.cpp)
target_include_directories(ruitong_tracking_node PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(ruitong_tracking_node
  rclcpp
  std_msgs
  ament_index_cpp
)

# Link RUITONG SDK library
target_link_directories(ruitong_tracking_node PRIVATE ${LIB_DIR})
target_link_libraries(ruitong_tracking_node ${LIB_NAME})

# Set runtime library path
set_target_properties(ruitong_tracking_node PROPERTIES
    BUILD_RPATH ${LIB_DIR}
    INSTALL_RPATH ${LIB_DIR}
)

# Link the generated messages
rosidl_target_interfaces(ruitong_tracking_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install
install(TARGETS
  ruitong_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install tool directory to share directory
install(DIRECTORY tool/
  DESTINATION share/${PROJECT_NAME}/tool
)

ament_package()
