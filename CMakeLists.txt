project(uma_vi_tools CXX)

cmake_minimum_required(VERSION 2.8.3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")


## Set CXX optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wpedantic -Wall -Wextra")

## Set build type to Release by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
else(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
message(STATUS "Flags: " ${CMAKE_CXX_FLAGS})

## Set output directory 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(Gflags REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${GFLAGS_INCLUDE_DIR}
)

add_executable(evaluate src/evaluate.cpp)
target_link_libraries(evaluate ${Boost_LIBRARIES} ${GFLAGS_LIBRARY})

add_executable(tum2asl src/tum2asl.cpp)
target_link_libraries(tum2asl ${Boost_LIBRARIES} ${GFLAGS_LIBRARY})

