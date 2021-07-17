
#######################################################################
# Physical Human Library CMAKE
#######################################################################

cmake_minimum_required(VERSION 2.6)

project(physhuman)
set(${CMAKE_PROJECT_NAME}_VERSION 1.00)

if(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#######################################################################
# Compilation Options
#######################################################################

option(BUILD_APPLICATIONS "Set when you want to build the application subprojects" ON)
option(USE_FLTK "Set when you want to include the FLTK library" ON)

if(WIN32)
  option(USE_KINECT "Set when you want to include the Kinect library" OFF)
endif(WIN32)

set(COMMON_PROJECT_INCLUDES "${CMAKE_SOURCE_DIR}/../cmake/includes" CACHE PATH "CMake common project includes directory")
if(EXISTS ${COMMON_PROJECT_INCLUDES}/projectinit.cmake)
	mark_as_advanced(COMMON_PROJECT_INCLUDES)

	#######################################################################
	# Project StartUp
	#######################################################################

	include(${COMMON_PROJECT_INCLUDES}/projectinit.cmake)
	
else(EXISTS ${COMMON_PROJECT_INCLUDES}/projectinit.cmake)
	message("Cannot find file projectinit.cmake. Please specify the correct CMake Common Includes folder")
endif(EXISTS ${COMMON_PROJECT_INCLUDES}/projectinit.cmake)


