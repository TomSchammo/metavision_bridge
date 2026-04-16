cmake_minimum_required(VERSION 3.8)
project(metavision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# requires C++ 20 for std::ranges and concepts
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(dvs_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(metavision_bridge src/main.cpp)
add_library(metavision_bridge_component SHARED src/main.cpp)
ament_target_dependencies(metavision_bridge_component
  rclcpp
  rclcpp_components
  event_camera_msgs
  event_camera_codecs
  dvs_msgs
)

rclcpp_components_register_node(metavision_bridge_component
  PLUGIN "EventBridgeNode"
  EXECUTABLE metavision_bridge
)

ament_export_targets(export_metavision_bridge_component)

install(TARGETS metavision_bridge_component
  EXPORT export_metavision_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
