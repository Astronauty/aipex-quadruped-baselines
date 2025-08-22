cmake_minimum_required(VERSION 3.8)
project(convex_mpc)

option(CXX "enable C++ compilation" ON)
if(CXX)
  enable_language(CXX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake") # Add to include FindGUROBI.cmake

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  pinocchio
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)


message(STATUS "GUROBI_INCLUDE_DIRS: ${GUROBI_INCLUDE_DIRS}")
message(STATUS "GUROBI_LIBRARY: ${GUROBI_LIBRARY}")
message(STATUS "GUROBI_CXX_LIBRARY: ${GUROBI_CXX_LIBRARY}")

# Eigen Headers
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})


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


### Executables ###
# set(MY_EXECUTABLES convex_mpc_controller)
set(MY_EXECUTABLES convex_mpc_controller gait_planner)


add_executable(convex_mpc_controller
  src/convex_mpc_node.cpp
  src/convex_mpc.cpp
  src/gurobi_utils.cpp
  src/simplified_quad_dynamics.cpp
  src/state_space.cpp
  src/transforms.cpp
  src/gait_planner.cpp
)

# add_executable(gait_planner
#   src/gait_planner.cpp
# )

### Specify URDF location ###
set(URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf")

# Pass it as a compile definition
target_compile_definitions(convex_mpc_controller PRIVATE
    PINOCCHIO_MODEL_DIR="${URDF_DIR}"
)

### Link Libraries for Executables ###
target_include_directories(convex_mpc_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GUROBI_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(convex_mpc_controller
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
  pinocchio::pinocchio
)

# target_include_directories(gait_planner PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${GUROBI_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
# )

# target_link_libraries(gait_planner
#   ${GUROBI_CXX_LIBRARY}
#   ${GUROBI_LIBRARY}
#   pinocchio::pinocchio
# )


ament_target_dependencies(convex_mpc_controller ${DEPENDENCY_LIST})
# ament_target_dependencies(gait_planner ${DEPENDENCY_LIST})


install(TARGETS
  convex_mpc_controller
  # gait_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

### Link Libraries for all executables ###
# foreach(exec ${MY_EXECUTABLES})
#   target_link_libraries(${exec}
#     ${GUROBI_LIBRARY}
#     ${GUROBI_CXX_LIBRARY}
#     Eigen3::Eigen
#   )
# endforeach()


ament_package()
