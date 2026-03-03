cmake_minimum_required(VERSION 3.10)
project(quadruped_ros2)

# Standard C++17 for Humble/Foxy-ready portability
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(drdds       REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Math Support (MPC requires BLAS/LAPACK)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
# Suppress noisy but non-fatal warnings to see real errors better
add_compile_options(-Wno-deprecated-declarations -Wno-format-truncation)

# ── Include Paths (Point to the shared quadruped logic) ──────────────────────
# Note: In Docker, we will mount or copy the Tophill logic here.
set(TOPHILL_ROOT "/workspace/quadruped-robot/quadruped")

include_directories(
  include/ros_stub
  include
  ${TOPHILL_ROOT}/include/quadruped
  ${TOPHILL_ROOT}/include
  ${TOPHILL_ROOT}
  ${TOPHILL_ROOT}/src
  ${TOPHILL_ROOT}/config
  ${TOPHILL_ROOT}/extern/unitree_legged_sdk/include
  ${TOPHILL_ROOT}/extern/unitree_legged_sdk_3.5/include
  ${TOPHILL_ROOT}/extern/tinynurbs/include
  ${TOPHILL_ROOT}/extern/TinyEKF/src
  ${TOPHILL_ROOT}/extern/qpOASES
  ${TOPHILL_ROOT}/extern/qpOASES/include
  ${TOPHILL_ROOT}/extern/QuadProgpp/src
  ${EIGEN3_INCLUDE_DIRS}
)

# ── Build Logic (Porting to ROS 2) ───────────────────────────────────────────
# Gather all source files for the MPC/WBC logic
file(GLOB_RECURSE MPC_SOURCES 
    "${TOPHILL_ROOT}/src/action/*.cpp"
    "${TOPHILL_ROOT}/src/controllers/*.cpp"
    "${TOPHILL_ROOT}/src/controllers/**/*.cpp"
    "${TOPHILL_ROOT}/src/dynamics/*.cpp"
    "${TOPHILL_ROOT}/src/estimators/*.cpp"
    "${TOPHILL_ROOT}/src/exec/*.cpp"
    "${TOPHILL_ROOT}/src/fsm/*.cpp"
    "${TOPHILL_ROOT}/src/gait/*.cpp"
    "${TOPHILL_ROOT}/src/planner/*.cpp"
    "${TOPHILL_ROOT}/src/ros/*.cpp"
    "${TOPHILL_ROOT}/src/utils/*.cpp"
    "${TOPHILL_ROOT}/src/robots/qr_robot.cpp"
    "${TOPHILL_ROOT}/src/robots/qr_motor.cpp"
    "${TOPHILL_ROOT}/extern/qpOASES/src/*.cpp"
    "${TOPHILL_ROOT}/extern/QuadProgpp/src/*.cpp"
    "${TOPHILL_ROOT}/extern/QuadProgpp/src/*.cc"
    "${TOPHILL_ROOT}/extern/TinyEKF/src/*.c"
)

# Exclude ROS1-specific files and irrelevant robot models
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*ros1_only.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*qr_robot_a1.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*qr_robot_go1.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*qr_robot_aliengo.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*qr_control2gazebo_msg.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*qr_robot_sim.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*gazebo_bridge.*")
list(FILTER MPC_SOURCES EXCLUDE REGEX ".*/QuadProgpp/src/main.cc")

# ── Executable (The Bridge Node) ─────────────────────────────────────────────
add_executable(mpc_runner 
    src/main.cpp
    src/ros_shim.cpp
    ${MPC_SOURCES}
)

target_link_libraries(mpc_runner
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
  -lpthread
  yaml-cpp
)

ament_target_dependencies(mpc_runner
  rclcpp
  drdds
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  yaml-cpp
)

# ── Installation ──────────────────────────────────────────────────────────────
install(TARGETS mpc_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
