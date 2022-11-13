cmake_minimum_required(VERSION 3.0.2)
# Use clang when we can
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-10")
#set(CMAKE_C_COMPILER "/usr/bin/clang-10")
project(rover_controller CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(BUILD_DOXYGEN_DOCS OFF)

find_package(Matlab REQUIRED) 

if (BUILD_DOXYGEN_DOCS)
  find_package(Doxygen REQUIRED)
endif()

set(PARAM_GEN_ENABLE ON)
if (PARAM_GEN_ENABLE)
  find_package(OpenMP REQUIRED)
  if (OPENMP_FOUND)
    message("OPENMP_FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()

find_package(catkin REQUIRED
        roscpp message_generation message_runtime std_msgs geometry_msgs joy tf2_msgs)

if (PARAM_GEN_ENABLE)
  add_compile_options(--std=c++17 -O3 -fopenmp )
else()
  add_compile_options(--std=c++17 -O3)
endif()

set(HOME $ENV{HOME})
set(CMAKE_CXX_STANDARD 17)
set(CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CONTROLLER_NAME ${CONTROLLER_NODE_NAME})
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake")

if (PARAM_GEN_ENABLE)
  set(IPOPT_INCLUDE_DIRS "/usr/local/include/coin-or/")
  include_directories(${IPOPT_INCLUDE_DIRS})
endif()

add_subdirectory(fmt)

if (PARAM_GEN_ENABLE)
  catkin_package(
    INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS}
    LIBRARIES rover_controller
    CATKIN_DEPENDS std_msgs geometry_msgs tf2_msgs)
else()
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rover_controller
    CATKIN_DEPENDS std_msgs geometry_msgs tf2_msgs)
endif()

# For files
file(GLOB_RECURSE ALL_SOURCE_FILES "src/*.cc" "include/*.hpp")
file(GLOB_RECURSE INCLUDE_FILES "include/*.hpp")
file(GLOB_RECURSE SRC_FILES_NO_EXEC "src/*.cc")
#message("CMAKE SOURCE DIR ${CMAKE_SOURCE_DIR}")
#message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
list(REMOVE_ITEM SRC_FILES_NO_EXEC "${CMAKE_CURRENT_LIST_DIR}/src/test_mex_comp.cc")
#message("SRC_FILES_NO_EXEC: ${SRC_FILES_NO_EXEC}")

set(CUSTOM_COMPILE_FLAGS " ")
set_source_files_properties( ${ALL_SOURCE_FILES} PROPERTIES COMPILE_FLAGS "${CUSTOM_COMPILE_FLAGS}" )
#message("${PROJECT_NAME}_EXPORTED_TARGETS: ${${PROJECT_NAME}_EXPORTED_TARGETS}")

function(add_rover_executable EXEC_TARGET_NAME EXEC_FNAME IS_MEX)
  if(IS_MEX)
    matlab_add_mex(NAME ${EXEC_TARGET_NAME} SRC ${EXEC_FNAME} ${SRC_FILES_NO_EXEC} ${INCLUDE_FILES})
  else()
    add_executable(${EXEC_TARGET_NAME} ${EXEC_FNAME} ${SRC_FILES_NO_EXEC} ${INCLUDE_FILES})
  endif()
  add_dependencies(${EXEC_TARGET_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  if(PARAM_GEN_ENABLE)
    target_include_directories(${EXEC_TARGET_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ipopt/include /usr/include/python3.8 )
    target_link_libraries(${EXEC_TARGET_NAME} ${catkin_LIBRARIES} fmt::fmt ipopt python3.8 pthread)
  else()
    target_include_directories(${EXEC_TARGET_NAME} PUBLIC src include ${catkin_INCLUDE_DIRS} /usr/include/python3.8 )
    target_link_libraries(${EXEC_TARGET_NAME} ${catkin_LIBRARIES} fmt::fmt python3.8)
  endif()
endfunction()

add_rover_executable(TEST_MEX src/test_mex_comp.cc TRUE)
