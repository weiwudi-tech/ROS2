cmake_minimum_required(VERSION 3.8)
project(filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-compare)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(f src/f.cpp)
ament_target_dependencies(f 
  rclcpp 
  std_msgs 
  geometry_msgs
)

add_executable(m src/middle.cpp)
ament_target_dependencies(m 
  rclcpp 
  std_msgs 
  geometry_msgs
)

add_executable(l src/low.cpp)
ament_target_dependencies(l 
  rclcpp 
  std_msgs 
  geometry_msgs
)

install(TARGETS
  f
  m
  l
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()