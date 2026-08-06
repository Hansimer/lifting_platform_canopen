cmake_minimum_required(VERSION 3.8)
project(lifting_platform_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)

# 生成 bin 文件（cogen 预处理 + dcfgen 生成）
cogen_dcf(lifting_platform)

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