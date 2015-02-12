cmake_minimum_required(VERSION 2.8.3)
project(opteq_wheelunits_01)

find_package(catkin REQUIRED
	COMPONENTS
		rose_common
		rose_conversions
		rose_geometry
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		opteq_wheelunits_01 
	CATKIN_DEPENDS
		rose_common
		rose_conversions
		rose_geometry
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(opteq_wheelunits_01 
	SHARED
	src/wheel_unit.cpp
)

target_link_libraries(opteq_wheelunits_01 ${catkin_LIBRARIES})
add_dependencies(opteq_wheelunits_01 ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
