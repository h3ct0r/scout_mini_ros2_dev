
cmake_minimum_required(VERSION 3.8)
project(fiber_tether)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMonitorStatus.msg"
  DEPENDENCIES geometry_msgs
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/battery_monitor_node.py
  scripts/dynamixel_motor.py
  scripts/motor_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  msg
)

#ament_package()