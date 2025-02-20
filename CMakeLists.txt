cmake_minimum_required(VERSION 3.16E)

project(gurobi-template C)

option(CXX "enable C++ compilation" ON)
include(ExternalProject)
if(CXX)
  enable_language(CXX)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
# list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

# search for gurobi
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})

# search for eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# # search for drake
# list(APPEND CMAKE_PREFIX_PATH /home/workstation4/daniel/aipex-quadruped-baselines/thirdparty/drake/tools/install/libdrake)
# set(DRAKE_PREFIX "${PROJECT_BINARY_DIR}/drake-prefix")


if(CXX)
  set(CMAKE_CXX_STANDARD 11)
endif()

# Add executable for gc_funcnonlinear
add_executable(gc_funcnonlinear examples/gc_funcnonlinear.cpp)
target_link_libraries(gc_funcnonlinear optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
target_link_libraries(gc_funcnonlinear ${GUROBI_LIBRARY} Eigen3::Eigen)

# Add executable for eigen_test
add_executable(eigen_test examples/eigen_test.cpp)
target_link_libraries(eigen_test Eigen3::Eigen)

# Add executable for mip1_c++
add_executable(mip1_c++ examples/mip1_c++.cpp)
target_link_libraries(mip1_c++ ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})


# Add executable for nl_mpc_controller
# add_executable(nl_mpc nl_mpc_controller/nl_mpc.cpp)
# target_link_libraries(nl_mpc ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

add_executable(quad_dynamics 
  simplified_quad_dynamics.cpp 
  state_space.cpp
  transforms.cpp)
target_link_libraries(quad_dynamics Eigen3::Eigen)


# add_executable("mip1_c++" examples/mip1_c++.cpp)
# if(CXX)
#   set(CMAKE_CXX_STANDARD 11)
#   # target_link_libraries(mip1_c++ optimized ${GUROBI_CXX_LIBRARY}
#   #                       debug ${GUROBI_CXX_DEBUG_LIBRARY})
# endif()

if(CXX)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  include(FeatureSummary)
  feature_summary(WHAT ALL)
endif()
