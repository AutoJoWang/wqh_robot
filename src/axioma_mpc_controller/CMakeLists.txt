cmake_minimum_required(VERSION 3.8)
project(axioma_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

# Find Eigen3 (通过 eigen3_cmake_module)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/robot_model.cpp
  src/path_handler.cpp
  src/mpc_controller.cpp
  src/mpc_controller_plugin.cpp
)

# Header files
set(HEADERS
  include/axioma_mpc_controller/robot_model.hpp
  include/axioma_mpc_controller/path_handler.hpp
  include/axioma_mpc_controller/mpc_controller.hpp
  include/axioma_mpc_controller/mpc_controller_plugin.hpp
)

# Create library
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# 添加编译定义（如果需要）
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 注册 nav2_core::Controller 插件到 ament 索引（仅 package.xml 的 export 不会生成索引，controller_server 无法发现插件）
pluginlib_export_plugin_description_file(nav2_core "mpc_controller_plugin.xml")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

ament_package()
