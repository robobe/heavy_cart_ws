cmake_minimum_required(VERSION 3.8)
project(heavy_cart_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
