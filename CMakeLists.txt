cmake_minimum_required(VERSION 3.8)
project(lifting_platform_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)

# 生成 bin 文件（cogen 预处理 + dcfgen 生成）
cogen_dcf(lifting_platform)

# CSP Init Node
add_executable(csp_init_node src/csp_init_node.cpp)
ament_target_dependencies(csp_init_node
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  canopen_interfaces
)
install(TARGETS csp_init_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他配置文件（master.dcf, ros2_controllers.yaml）
install(FILES
  config/ros2_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch
  meshes
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()