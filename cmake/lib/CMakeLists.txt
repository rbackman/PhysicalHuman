

# SkelBullet build CMakeList
set(SRC_FILES "")
set(SRC_TEMP "")
file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Utlities/*.cpp")
set(SRC_FILES ${SRC_TEMP})
file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Graphics/*.cpp")
set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})
file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Physics/*.cpp")
set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})
file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Human/*.cpp")
set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})
file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/include/*.cpp")
set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})

if(USE_KINECT)
  file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Kinect/*.cpp")
  set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})
else(USE_KINECT)
  list(REMOVE_ITEM SRC_FILES "${physhuman_SOURCE_DIR}/include/kinect_fluid.cpp")
endif(USE_KINECT)

if(USE_FLTK)
  file(GLOB SRC_TEMP "${physhuman_SOURCE_DIR}/src/Window/*.cpp")
  set(SRC_FILES ${SRC_FILES} ${SRC_TEMP})
endif(USE_FLTK)

file(GLOB HEADER_FILES "${physhuman_SOURCE_DIR}/include/*.h")

file(GLOB HEADER_GLOB_HAPP_FILES "${physhuman_SOURCE_DIR}/include/human_app*.h")

if(NOT USE_KINECT)
  file(GLOB HEADER_GLOB_KINECT_FILES "${physhuman_SOURCE_DIR}/include/kinect*.h")
endif(NOT USE_KINECT)

if(NOT USE_FLTK)
  file(GLOB HEADER_GLOB_HWINDOW_FILES "${physhuman_SOURCE_DIR}/include/human_window*.h")
endif(NOT USE_FLTK)

list(REMOVE_ITEM HEADER_FILES ${HEADER_GLOB_KINECT_FILES} ${HEADER_GLOB_HAPP_FILES} ${HEADER_GLOB_HWINDOW_FILES})


include_directories("${physhuman_SOURCE_DIR}/include/")


# Add needed definitions
#add_definitions(-D_MT -D_USRDLL)

# linking for shared compilation
add_library(physhuman ${HEADER_FILES} ${SRC_FILES})

# External dependancies

# the order matters on linux 
set(LIBRARIES "${OPENGL_LIBRARIES}")

if(WIN32)
  set(LIBRARIES ${LIBRARIES} ${WIN32_SOCKET_LIBS})
endif(WIN32)
  
if(USE_FLTK)
  if(WIN32)
    set(LIBRARIES ${LIBRARIES} ${FLTK2_LIBRARIES_DEBUG})
  endif(WIN32)  
  set(LIBRARIES ${LIBRARIES} ${GRAPHSIMFL_LIBRARIES} ${FLTK2_LIBRARIES})
endif(USE_FLTK)

if(USE_KINECT)
  set(LIBRARIES ${LIBRARIES}
    ${KINECTSDK_LIBRARIES}
    )
endif(USE_KINECT)

set(LIBRARIES ${LIBRARIES}
    ${GRAPHSIM_LIBRARIES}
    ${GRAPHSIM_SDKS_LIBRARIES}
  )

target_link_libraries(physhuman ${LIBRARIES})

# install properties
install(TARGETS physhuman DESTINATION lib)
install(DIRECTORY ${physhuman_SOURCE_DIR}/data DESTINATION . PATTERN ".svn*" EXCLUDE)
install(DIRECTORY ${physhuman_SOURCE_DIR}/models DESTINATION . PATTERN ".svn*" EXCLUDE)
install(FILES ${HEADER_FILES} DESTINATION ./include )


