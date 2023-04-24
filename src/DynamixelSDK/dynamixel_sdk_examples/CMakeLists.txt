cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

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

set(dynamixel /home/ros2access/kaya_ws/install)
set(custom_interfaces /home/ros2access/kaya_ws/install())
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED PATHS ${dynamixel})
find_package(dynamixel_sdk_custom_interfaces REQUIRED PATHS ${custom_interfaces})
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)

include_directories(include)


# Build
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp 
)

add_executable(my_read_write src/my_read_write.cpp)
ament_target_dependencies(my_read_write dynamixel_sdk dynamixel_sdk_custom_interfaces rclcpp)

add_executable(motorcontroller src/motorcontroller.cpp)
ament_target_dependencies(motorcontroller dynamixel_sdk rclcpp dynamixel_sdk_custom_interfaces)

add_executable(relativespeeds src/relativespeeds.cpp)
target_link_libraries(relativespeeds Eigen3::Eigen)
ament_target_dependencies(relativespeeds rclcpp dynamixel_sdk_custom_interfaces geometry_msgs)

add_executable(dynamixelcontroller src/dynamixelcontroller.cpp)
ament_target_dependencies(dynamixelcontroller rclcpp dynamixel_sdk geometry_msgs dynamixel_sdk_custom_interfaces)

# Install
install(TARGETS
  read_write_node
  my_read_write
  motorcontroller
  relativespeeds
  dynamixelcontroller
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
