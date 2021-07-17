
#######################################################################
# Code Subdirectory
#######################################################################

add_subdirectory( lib )

if(BUILD_APPLICATIONS AND USE_FLTK AND USE_KINECT)
  add_subdirectory( app )
endif(BUILD_APPLICATIONS AND USE_FLTK AND USE_KINECT)
