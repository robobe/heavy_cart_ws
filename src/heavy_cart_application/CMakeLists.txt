cmake_minimum_required(VERSION 3.8)
project(heavy_cart_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


set (NODES
  heavy_cart_application/mav_node.py
)

set (FILES
  heavy_cart_application/mavlink.py
)

install(PROGRAMS
    ${NODES}
    ${FILES}
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
