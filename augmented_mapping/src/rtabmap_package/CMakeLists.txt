cmake_minimum_required(VERSION 3.8)
project(rtabmap_package)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtabmap_ros REQUIRED)


# If you have any C++ executables, uncomment and modify the following:
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)


# Include directories (uncomment if using custom C++ code)
# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )


# If you have C++ executables, uncomment and add them here
# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
# ament_target_dependencies(my_node rclcpp sensor_msgs std_msgs)


# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install Python files (if any scripts are directly executed)
install(PROGRAMS
  launch/rtabmap_launch.py
  DESTINATION lib/${PROJECT_NAME}
)


# Mark other files for installation (uncomment as needed)
# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )


ament_package()
