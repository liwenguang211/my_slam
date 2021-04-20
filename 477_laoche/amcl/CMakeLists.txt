cmake_minimum_required(VERSION 3.5)

# include directories
include_directories(
	include
	${PROJECT_SOURCE_DIR}
)
link_directories(
	include
)
INCLUDE_DIRECTORIES("/usr/include/eigen3")
add_compile_options(-w)
file(GLOB AMCL_SRC "*.cpp")
add_compile_options(-g)
add_library (AMCLCommon ${AMCL_SRC})
