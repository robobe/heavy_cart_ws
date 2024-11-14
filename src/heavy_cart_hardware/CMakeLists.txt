cmake_minimum_required(VERSION 3.8)
project(heavy_cart_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(mcp4725_test tests/mcp4725.cpp src/mcp4725.cpp)
target_include_directories(mcp4725_test PRIVATE include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(
  heavy_cart_arduino
  SHARED
  src/heavy_cart_hardware.cpp
)
target_compile_features(heavy_cart_arduino PUBLIC cxx_std_17)
target_include_directories(heavy_cart_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/heavy_cart_arduino>
)
ament_target_dependencies(
  heavy_cart_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS heavy_cart_arduino
  EXPORT export_heavy_cart_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface heavy_cart_arduino.xml)
ament_export_targets(export_heavy_cart_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
