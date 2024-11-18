cmake_minimum_required(VERSION 3.3)

project(gurobi-template C)

option(CXX "enable C++ compilation" ON)

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


# search for eigen
message(${PROJECT_SOURCE_DIR}/thirdparty/eigen)
message(${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/thirdparty/eigen/cmake)
message(${CMAKE_PREFIX_PATH})
# add_subdirectory(thirdparty/eigen)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
add_executable("eigen_test" eigen_test.cpp)

# list source files here
set(sources mip1_c++.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${sources})

if(CXX)
  set(CMAKE_CXX_STANDARD 11)
  target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY}
                        debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})
target_link_libraries("eigen_test" Eigen3::Eigen)


if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  include(FeatureSummary)
  feature_summary(WHAT ALL)
endif()
