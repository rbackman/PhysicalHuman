#######################################################################
# Core dependencies
#######################################################################

#overriding initial PATH ... just for the first instance
if(NOT RESET_DEP_SEARCH_DIR)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../../install/${CMAKE_PROJECT_NAME}" CACHE PATH "Graphsim Devices install prefix" FORCE)
  set(GRAPHSIM_SDKS_DIR "${CMAKE_BINARY_DIR}/../../install/sdks" CACHE PATH "Graphsim SDKs installation directory" FORCE)
  set(GRAPHSIM_DIR "${CMAKE_BINARY_DIR}/../../install/graphsim" CACHE PATH "Graphsim installation directory" FORCE)
  set(GRAPHSIMFL_DIR "${CMAKE_BINARY_DIR}/../../install/graphsim" CACHE PATH "Graphsim FL installation directory" FORCE)
  mark_as_advanced( GRAPHSIMFL_DIR )
  if(WIN32)
    set(FLTK2_DIR "${CMAKE_BINARY_DIR}/../../install/fltk" CACHE PATH "Graphsim FLTK2 installation directory" FORCE)
	mark_as_advanced( FLTK2_DIR )
  endif(WIN32)
  
  set(RESET_DEP_SEARCH_DIR ON CACHE BOOL "Set when you want to reset the dependencies directory according to the installation path" FORCE)
  mark_as_advanced( RESET_DEP_SEARCH_DIR )
endif(NOT RESET_DEP_SEARCH_DIR)

# Find OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
	message("OpenGL FOUND")
	include_directories( ${OpenGL_INCLUDE_DIRS} )
	link_directories( ${OPENGL_LIBRARY_DIRS} )
else (OPENGL_FOUND)
	message("OPENGL NOT FOUND: Setting default libraries")
	set(OPENGL_gl_LIBRARY opengl32)
	set(OPENGL_glu_LIBRARY glu32)
endif (OPENGL_FOUND)
mark_as_advanced(OpenGL)

# Find Graphsim SDKS
set(LOOKFOR_ODE TRUE)
find_package(GraphsimSDKS REQUIRED)
if(GRAPHSIM_SDKS_FOUND)
	message("GraphsimSDKS FOUND")
else (GRAPHSIM_SDKS_FOUND)
	message("GraphsimSDKS NOT FOUND: Specify the intallation directory")
endif (GRAPHSIM_SDKS_FOUND)

# Find Graphsim Toolkit
find_package(Graphsim REQUIRED)
if(GRAPHSIM_FOUND)
	message("Graphsim Toolkit FOUND")
else(GRAPHSIM_FOUND)
	message("Graphsim Toolkit NOT FOUND: Specify the intallation directory")
endif(GRAPHSIM_FOUND)

# Find Kinect SDK Toolkit
if(USE_KINECT)
  find_package(KinectSDK REQUIRED)
  if(KINECTSDK_FOUND)
	message("Kinect SDK FOUND")
  else(KINECTSDK_FOUND)
	message("Kinect SDK NOT FOUND: Specify the intallation directory")
  endif(KINECTSDK_FOUND)
endif(USE_KINECT)

# Find Graphsim FLTK2
# Find Graphsim FLTK2
if(USE_FLTK)
	find_package(GraphsimFLTK2 REQUIRED)
	if(GRAPHSIMFLTK2_FOUND)
		message("FLTK2 FOUND")
		set(GRAPHSIMFLTK2_INTERNAL_CHECK "YES")
	else (GRAPHSIMFLTK2_FOUND)
		message("FLTK2 NOT FOUND: Specify the intallation directory")
		set(GRAPHSIMFLTK2_INTERNAL_CHECK "")
	endif (GRAPHSIMFLTK2_FOUND)
	
	find_package(GraphsimFL REQUIRED)
	if(GRAPHSIMFL_FOUND)
		message("GraphsimFL FOUND")
		set(GRAPHSIMFL_INTERNAL_CHECK "YES")
	else (GRAPHSIMFL_FOUND)
		message("GraphsimFL NOT FOUND: Specify the intallation directory")
		set(GRAPHSIMFL_INTERNAL_CHECK "")
	endif (GRAPHSIMFL_FOUND)
	mark_as_advanced( CLEAR GRAPHSIMFL_DIR )
else(USE_FLTK)
	set(GRAPHSIMFLTK2_INTERNAL_CHECK "YES")
	set(GRAPHSIMFL_INTERNAL_CHECK "YES")
endif(USE_FLTK)

set( SAFETY_CHECK "" )
if(OPENGL_FOUND AND GRAPHSIM_FOUND AND 
  GRAPHSIMFLTK2_INTERNAL_CHECK AND 
  GRAPHSIMFL_INTERNAL_CHECK AND GRAPHSIM_SDKS_FOUND)
	set( SAFETY_CHECK "YES" )
endif(OPENGL_FOUND AND GRAPHSIM_FOUND AND 
  GRAPHSIMFLTK2_INTERNAL_CHECK AND 
  GRAPHSIMFL_INTERNAL_CHECK AND GRAPHSIM_SDKS_FOUND)


