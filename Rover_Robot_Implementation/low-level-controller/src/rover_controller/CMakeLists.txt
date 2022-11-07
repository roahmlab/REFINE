cmake_minimum_required(VERSION 3.0.2)
project(rover_controller CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(PARAM_GEN_ENABLE OFF)
if (PARAM_GEN_ENABLE)
  find_package(OpenMP REQUIRED)
  if (OPENMP_FOUND)
    message("OPENMP_FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()

#set(CXX g++-10)
#set(CC gcc-10)
set(CXX clang++-10)
set(CC clang-10)
## Compile as C++11, supported in ROS Kinetic and newer

if (PARAM_GEN_ENABLE)
  add_compile_options(--std=c++17 -O3 -fopenmp )
else()
  add_compile_options(--std=c++17 -O3)
endif()
set(HOME $ENV{HOME})
set(CMAKE_CXX_STANDARD 17)
set(CXX_STANDARD 17)
set(CONTROLLER_NAME ${CONTROLLER_NODE_NAME})
set(ONLINE_NODE_NAME "parameter_generation_node")
set(CONTROLLER_NODE_NAME "rover_controller_node")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake")
message(CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR})
message(CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_subdirectory(fmt)

find_package(catkin REQUIRED
  roscpp
  message_generation
  message_runtime
  std_msgs
  jsk_recognition_msgs
  geometry_msgs
  joy
  vesc_msgs
  tf2_msgs
  rover_control_msgs
  state_estimator
)
if (PARAM_GEN_ENABLE)
  set(IPOPT_INCLUDE_DIRS ${HOME}/src/coinbrew/coinbrew/dist/include/coin-or)
endif()
catkin_package(
  INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS}
  LIBRARIES rover_controller
  CATKIN_DEPENDS std_msgs geometry_msgs vesc_msgs jsk_recognition_msgs rover_control_msgs tf2_msgs state_estimator
  # DEPENDS fmt
)
# For files
file(GLOB_RECURSE SRC_DIR "src/*.cc" "include/*.h")
set_source_files_properties(
  ${SRC_DIR}
  PROPERTIES
  #COMPILE_FLAGS " -Weverything -Wall -Wno-c++98-compat -Wno-c++98-compat-pedantic "
)
include_directories(
 include
  fmt
  ${catkin_INCLUDE_DIRS}
) 
add_executable(${CONTROLLER_NODE_NAME} src/rover_controller.cc)
if (PARAM_GEN_ENABLE)
  add_executable(${ONLINE_NODE_NAME} src/parameter_generation.cc)
  target_include_directories(${ONLINE_NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${IPOPT_INCLUDE_DIRS}>
    /usr/include/python3.8
    #$<BUILD_INTERFACE:/usr/include/coin>
  )
endif()
add_dependencies(
  ${CONTROLLER_NODE_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
if (PARAM_GEN_ENABLE)
  add_dependencies(
    ${ONLINE_NODE_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )
endif()
target_link_libraries(${CONTROLLER_NODE_NAME}
  ${catkin_LIBRARIES}
  fmt::fmt
)
if (PARAM_GEN_ENABLE)
  target_link_libraries(${ONLINE_NODE_NAME}
    ${catkin_LIBRARIES}
    fmt::fmt
    python3.8
    ipopt
    pthread
  )
endif()

#############
## Testing ##
#############

#include(GoogleTest)
#
#function (addtest fname)
#  set(fpath ${PROJECT_NAME}_${fname})
#  message(STATUS "hey ${fpath}")
#  catkin_add_executable_with_gtest(${fpath} test/${fname}.cc)
#  add_dependencies(${fpath} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#  #target_link_libraries(${fpath} ${catkin_LIBRARIES} gtest_main)
#  target_link_libraries(${fpath} ${catkin_LIBRARIES} fmt::fmt gtest_main)
#  gtest_discover_tests(${fpath})
#endfunction()
## Add gtest based cpp test target and link libraries


# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
