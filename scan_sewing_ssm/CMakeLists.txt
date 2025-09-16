cmake_minimum_required(VERSION 3.0.2)
project(scan_sewing_ssm)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_CXX_EXTENSIONS OFF) 

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS}
  include
)

link_directories(${CMAKE_SOURCE_DIR}/lib)


catkin_package()

file(GLOB_RECURSE MOC_H RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/scan_sewing_ssm/*.h)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)



add_executable(scan_sewing_ssm ${MOC_H} ${SOURCES})

add_dependencies(scan_sewing_ssm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_sewing_ssm ${catkin_LIBRARIES})