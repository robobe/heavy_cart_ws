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
find_package(ament_cmake_python REQUIRED)

# mcp4725 demo
add_executable(mcp4725_test tests/mcp4725.cpp src/mcp4725.cpp)
target_include_directories(mcp4725_test PRIVATE include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(
  heavy_cart_i2c_diff_hw
  SHARED
  src/heavy_cart_i2c_diff_hatdware.cpp
  src/mcp4725.cpp
)
target_compile_features(heavy_cart_i2c_diff_hw PUBLIC cxx_std_17)
target_include_directories(heavy_cart_i2c_diff_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/heavy_cart_i2c_diff_hw>
)
ament_target_dependencies(
  heavy_cart_i2c_diff_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS heavy_cart_i2c_diff_hw
  EXPORT export_heavy_cart_i2c_diff_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set (SCRIPTS
  tests/twist_stamper.py
)

install(PROGRAMS
    ${SCRIPTS}
DESTINATION lib/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(hardware_interface heavy_cart_i2c_hw.xml)
ament_export_targets(export_heavy_cart_i2c_diff_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
