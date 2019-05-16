cmake_minimum_required (VERSION 2.6)
project(Komondor)


set(COST_PATH ${PROJECT_SOURCE_DIR}/COST/cxx)

find_package(PkgConfig REQUIRED) 
pkg_check_modules(GLIB REQUIRED glib-2.0)

# add the command to generate the source code
add_custom_command (
	OUTPUT ${PROJECT_SOURCE_DIR}/main/komondor_main.cxx
 	COMMAND ${COST_PATH} komondor_main.cc
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/main
	DEPENDS ${PROJECT_SOURCE_DIR}/main/komondor_main.cc ${PROJECT_SOURCE_DIR}/main/compcxx_komondor_main.h
)


add_executable(komondor_main ${PROJECT_SOURCE_DIR}/main/komondor_main.cxx) 

include_directories(${GLIB_INCLUDE_DIRS})
target_link_libraries(komondor_main ${GLIB_LIBRARIES})


