

# SkelBullet build CMakeList
file(GLOB SRC_FILES "${physhuman_SOURCE_DIR}/src/App/*.cpp")
file(GLOB HEADER_FILES "${graphsimdev_SOURCE_DIR}/include/human_app*.h")


include_directories("${physhuman_SOURCE_DIR}/include/")

# Add needed definitions
#add_definitions(-D_MT -D_USRDLL)

# External dependancies
set(LIBRARIES
  physhuman
)

# Creating the Binary
if(WIN32)
  add_executable(phapp WIN32 ${SRC_FILES} ${HEADER_FILES} )
  set_target_properties(phapp PROPERTIES LINK_FLAGS /ENTRY:"mainCRTStartup" )
else(WIN32)
  add_executable(phapp ${SRC_FILES} ${HEADER_FILES} )
endif(WIN32)

target_link_libraries(phapp ${LIBRARIES})

# Compilation dependencies
add_dependencies( phapp physhuman )

# install properties
install(TARGETS phapp DESTINATION bin)

