cmake_minimum_required (VERSION 2.6)
project (panda_dyn_model_example)

## environment settings
set(CMAKE_COLOR_MAKEFILE ON)

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


###########
## Build ##
###########
message("Configuring main Project: " ${PROJECT_NAME})
message("CMAKE_BUILD_TYPE")
message(STATUS ${CMAKE_BUILD_TYPE})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_FRANKA_MODEL
  src/FrictionTorque.cpp
  src/CoriolisMatrix.cpp
  src/GravityVector.cpp
  src/MassMatrix.cpp
)

## Declare a c++ library
set(FRANKA_MODEL_LIBRARY franka_model)
add_library(${FRANKA_MODEL_LIBRARY} SHARED ${SRC_FRANKA_MODEL})

## Declare a C++ executable
add_executable(
  Model_example src/Model_example.cpp
)

target_link_libraries( 
  Model_example
  ${FRANKA_MODEL_LIBRARY} 
  ${EIGEN3_INCLUDE_DIRS}
)

