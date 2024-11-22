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

# message(${GUROBI_INCLUDE_DIRS})

# search for drake
list(APPEND CMAKE_PREFIX_PATH /home/workstation4/daniel/aipex-quadruped-baselines/thirdparty/drake/tools/install/libdrake)
set(DRAKE_PREFIX "${PROJECT_BINARY_DIR}/drake-prefix")





# search for eigen
# list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/thirdparty/eigen/cmake) 

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen)

ExternalProject_Add(drake
  DEPENDS eigen fmt spdlog
  URL https://github.com/RobotLocomotion/drake/archive/master.tar.gz
  # Or from a commit (download and use "shashum -a 256 'xxx.tar.gz'" on it to
  # get the URL_HASH.
  # URL https://github.com/RobotLocomotion/drake/archive/65c4366ea2b63278a286b1e22b8d464d50fbe365.tar.gz
  # URL_HASH SHA256=899d98485522a7cd5251e50a7a6b8a64e40aff2a3af4951a3f0857fd938cafca
  TLS_VERIFY ON
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILEER}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
    -DPYTHON_EXECUTABLE:FILEPATH=${Python3_EXECUTABLE}
    -DWITH_USER_EIGEN:BOOLEAN=ON
    -DWITH_USER_FMT:BOOLEAN=ON
    -DWITH_USER_SPDLOG:BOOLEAN=ON
  PREFIX "${DRAKE_PREFIX}"
  BINARY_DIR "${PROJECT_BINARY_DIR}/thirdparty/drake"
  BUILD_ALWAYS ON
)
add_executable("eigen_test" eigen_test.cpp)

# list source files here
set(sources mip1_c++.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${sources})

if(CXX)
  set(CMAKE_CXX_STANDARD 11)
  target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY} drake::drake
                        debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})
target_link_libraries("eigen_test" Eigen3::Eigen)


if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  include(FeatureSummary)
  feature_summary(WHAT ALL)
endif()
