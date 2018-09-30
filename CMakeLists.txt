###############################################################
######                                                   ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(pid_node)

#########################################
###### Loading 3rd party libraries.######
#########################################
add_definitions(-std=c++11)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "release")
ENDIF(NOT DEFINED CMAKE_BUILD_TYPE)

## Load PCL
find_package(PCL 1.8 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

## Load OpenRAVE
find_package(OpenRAVE)
if(${OpenRAVE_FOUND})
	add_definitions(-DHAS_OPENRAVE)
else(${OpenRAVE_FOUND})
	MESSAGE(WARNING "OpenRAVE NOT FOUND")
endif(${OpenRAVE_FOUND})

## Catkin
find_package(catkin REQUIRED COMPONENTS
			std_msgs
			sensor_msgs
			geometry_msgs
			std_srvs
			tf
			)

add_service_files(
  FILES
  Float32Param.srv
)

generate_messages(
    DEPENDENCIES std_msgs 
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs std_srvs
)

find_package(OpenCL) 

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)

#########################################
######			Executables		   ######
#########################################
## mav controller
file(GLOB_RECURSE PID_NODE_FILES "pid_node/*.cpp" "pid_node/*.h" "pid_node/*.inl" )
add_executable			(pid_node ${PID_NODE_FILES})

target_include_directories	(pid_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries		(pid_node ${catkin_LIBRARIES})

## pid tune gui
file(GLOB_RECURSE PID_TUNE_GUI_FILES "pid_tune_gui/*.cpp" "pid_tune_gui/*.h" "pid_tune_gui/*.inl" )
add_executable				(pid_tune_gui ${PID_TUNE_GUI_FILES})

target_include_directories	(pid_tune_gui PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries		(pid_tune_gui ${catkin_LIBRARIES})

target_link_libraries		(pid_tune_gui Qt5::Widgets)
target_link_libraries		(pid_tune_gui Qt5::PrintSupport)

