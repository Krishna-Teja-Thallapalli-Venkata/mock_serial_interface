cmake_minimum_required(VERSION 3.8)
project(mock_serial_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

add_library(mock_serial_interface SHARED
  src/mock_driver.cpp
  src/mock_hardware_interface.cpp
)

target_include_directories(mock_serial_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mock_serial_interface PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  hardware_interface::hardware_interface
)

pluginlib_export_plugin_description_file(hardware_interface mock_serial_interface.xml)

install(
  TARGETS mock_serial_interface
  EXPORT export_mock_serial_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mock_serial_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY description launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mock_serial_interface)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
