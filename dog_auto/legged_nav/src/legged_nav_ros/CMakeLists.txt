cmake_minimum_required(VERSION 3.16)
project(legged_nav_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # 生成 compile_commands.json 供 clangd(IDE 解析 ROS2 头)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# ============================================================================
# cmd_bridge: 订阅 /cmd_vel(Twist) → seqlock 写共享内存 /legged_nav_cmd。
#   需 legged_nav 的 nav_cmd_shm.hpp(同工作区相对路径, 布局单一真源, 不复制) + rt(shm_open)。
# ============================================================================
add_executable(cmd_bridge src/cmd_bridge.cpp)
target_include_directories(cmd_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../legged_nav/include)   # nav_cmd_shm.hpp
ament_target_dependencies(cmd_bridge rclcpp geometry_msgs)
target_link_libraries(cmd_bridge rt)

# ============================================================================
# sbus_teleop: 读 SBUS 串口 → 摇杆映射 Vx/Vy/ω → 发布 /cmd_vel。
#   链复活的 SBUS 读取(本包 include/src, 复制自 legged_vmc)。
# ============================================================================
add_executable(sbus_teleop
  src/sbus_teleop.cpp
  src/sbus_reader.cpp
  src/sbus_serial_linux.cpp)
target_include_directories(sbus_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(sbus_teleop rclcpp geometry_msgs)
target_link_libraries(sbus_teleop Threads::Threads)

install(TARGETS cmd_bridge sbus_teleop
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
