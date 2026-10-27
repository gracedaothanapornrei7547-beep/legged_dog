cmake_minimum_required(VERSION 3.16)
project(legged_nav LANGUAGES CXX)

# ============================================================================
# 编译选项
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

include_directories(include)

# ============================================================================
# C++ 核心静态库
# 4 个组件：calibration / kinematics / can_bridge / contact_detector
# ============================================================================
add_library(motor_test_core STATIC
  # ---- interface (mechanics & kinematic chain) ----
  src/kinematics/leg_calibration.cpp
  src/kinematics/leg_kinematics.cpp
  src/kinematics/baseline.cpp           # control_chain + 电机 PD config
  # ---- hardware ----
  src/driver/can_bridge.cpp          # 单通道 SocketCAN 底层库 (被 usb_can 调用)
  src/driver/usb_can.cpp             # 4 腿 USB-CAN 复合驱动 (按 USB 物理口绑腿)
  src/driver/imu_reader.cpp
  # ---- estimation ----
  src/estimation/contact_detector.cpp
  src/estimation/estimator.cpp
  # ---- gait planning (任务赛: 仅走路, 跳跃已删) ----
  src/planning/gait.cpp
  # ---- wbc (force control stack) ----
  src/dynamics/body_posture.cpp             # estimator → wrench_des
  src/dynamics/force_alloc.cpp              # wrench → per-leg F_body (heuristic)
  src/dynamics/force_control.cpp            # τ = J^T·F per leg
  # ---- task (任务层: 硬件编排) ----
  src/task/hardware_manager.cpp        # HardwareManager: 集中持有 IMU/SBUS/CAN + preflight
  src/task/command_source.cpp        # ★命令源: shm nav 自控(读 /legged_nav_cmd, Vx/Vy/ω→步长)
  src/task/dog_control.cpp           # DogControl: 整个 per-tick 控制环 tick()+print_summary()
)
target_include_directories(motor_test_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_test_core PUBLIC Eigen3::Eigen Threads::Threads rt)  # rt: shm_open (ShmCommandSource)
target_compile_definitions(motor_test_core PRIVATE EIGEN_DONT_PARALLELIZE)
# 静态库要能被链进 .so（外部 pybind11 桥等），所有 .o 必须 -fPIC。
# 对 trot_quad_main 等本包二进制零影响。
set_target_properties(motor_test_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# 可执行：四腿 trot (任务赛 nav 行走底盘)
# ============================================================================
add_executable(nav_quad src/nav_quad_main.cpp)
target_link_libraries(nav_quad PRIVATE motor_test_core)
target_compile_definitions(nav_quad PRIVATE EIGEN_DONT_PARALLELIZE)
set(MT_EXECUTABLES nav_quad)

# 单腿电机 CAN 诊断/标定工具（Function_can_test.py 的 C++ 版）
# 自包含 SocketCAN，pthread；★ 链 motor_test_core 拉 LegKinematics + get_calibration + Eigen，
# read 模式打印 FK 解算的足端 3D 坐标用。
add_executable(function_can_test tools/function_can_test.cpp)
target_link_libraries(function_can_test PRIVATE motor_test_core Threads::Threads)
list(APPEND MT_EXECUTABLES function_can_test)

# shm 命令喂数测试工具（块3a 验证 nav 通路, 无需 ROS2 bridge）。仅需 nav_cmd_shm.hpp(头) + rt。
add_executable(shm_feed tools/shm_feed.cpp)
target_link_libraries(shm_feed PRIVATE rt)
list(APPEND MT_EXECUTABLES shm_feed)

# shm 读端监听工具（临时「狗接口」接收端）：只读 /legged_nav_cmd，用与 ShmCommandSource
# 完全一致的 seqlock+0.3s 看门狗，验证 nav→cmd_bridge→狗接口 链路格式。仅需 nav_cmd_shm.hpp(头) + rt。
add_executable(shm_monitor tools/shm_monitor.cpp)
target_link_libraries(shm_monitor PRIVATE rt)
list(APPEND MT_EXECUTABLES shm_monitor)

# 无运动 CAN 电机扫描器(7845HS 移植: 定"哪条 bus=哪条腿"; 只发只读上报帧, 不动电机)
add_executable(can_scan tools/can_scan.cpp)
target_link_libraries(can_scan PRIVATE motor_test_core Threads::Threads)
list(APPEND MT_EXECUTABLES can_scan)


# IMU 独立测试（只读 IMU，不动电机不开 CAN）
# 源文件存在才编——本工作树同步时可能未带 imu_test_main.cpp，缺则跳过不卡构建。
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/imu_test_main.cpp)
  add_executable(imu_test src/imu_test_main.cpp)
  target_link_libraries(imu_test PRIVATE motor_test_core)
  list(APPEND MT_EXECUTABLES imu_test)
else()
  message(STATUS "motor_test: 跳过 imu_test（缺 src/imu_test_main.cpp）")
endif()

# ============================================================================
# 安装
# ============================================================================
install(TARGETS ${MT_EXECUTABLES}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS motor_test_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ============================================================================
# 测试（gtest 单元）
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 测试源在本工作树缺失时跳过（与上面 imu_test 同理）。
  # 否则缺一个 .cpp 就让 CMake 配置失败、连主程序都编不了。
  # 每项 "目标名:源文件"（foreach 内按 ":" 拆开）。
  set(_mt_gtests
    "test_pod_types:test/unit/test_pod_types.cpp"
    "test_leg_calibration:test/unit/test_leg_calibration.cpp"
    "test_leg_kinematics:test/unit/test_leg_kinematics.cpp"
    "test_can_protocol:test/unit/test_slice4_units.cpp"   # CAN 协议层测试
  )
  foreach(_entry IN LISTS _mt_gtests)
    string(REPLACE ":" ";" _pair "${_entry}")
    list(GET _pair 0 _tname)
    list(GET _pair 1 _tsrc)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_tsrc}")
      ament_add_gtest(${_tname} ${_tsrc})
      target_link_libraries(${_tname} motor_test_core)
    else()
      message(STATUS "motor_test: 跳过 ${_tname}（缺 ${_tsrc}）")
    endif()
  endforeach()
endif()

ament_package()
