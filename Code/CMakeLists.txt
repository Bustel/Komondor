cmake_minimum_required (VERSION 2.6)
project(Komondor)
set(CMAKE_BUILD_TYPE Debug)

set(COST_PATH ${PROJECT_SOURCE_DIR}/COST/compc++)

find_package(PkgConfig REQUIRED) 
pkg_check_modules(GLIB REQUIRED glib-2.0)


file(GLOB CXX_SRCS 
	"${COST_PATH}/*.cpp"
	"${COST_PATH}/*.c"
	"${COST_PATH}/*.h"
)

add_executable(cxx ${CXX_SRCS} ${COST_PATH}/lex.yy.cc)


add_custom_command(
	OUTPUT ${COST_PATH}/lex.yy.cc
	COMMAND flex -o lex.yy.cc ${COST_PATH}/lexer.l
	WORKING_DIRECTORY ${COST_PATH}
	DEPENDS ${COST_PATH}/lexer.l
)

# add the command to generate the source code
add_custom_command (
	OUTPUT ${PROJECT_SOURCE_DIR}/main/komondor_main.cxx
	COMMAND cxx -I${PROJECT_SOURCE_DIR}/include komondor_main.cc
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/main
	DEPENDS ${PROJECT_SOURCE_DIR}/main/komondor_main.cc ${PROJECT_SOURCE_DIR}/main/compcxx_komondor_main.h
)


add_executable(komondor_main ${PROJECT_SOURCE_DIR}/main/komondor_main.cxx) 

include_directories(${GLIB_INCLUDE_DIRS})
target_link_libraries(komondor_main ${GLIB_LIBRARIES})


