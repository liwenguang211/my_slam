project(map_creator)

cmake_minimum_required(VERSION 3.10)

find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(suitesparse QUIET)
if(NOT SuiteSparse_FOUND)
  set(SuiteSparse_LIBRARIES blas lapack cholmod cxsparse)
endif()

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

add_compile_options(-w)
include_directories(
	include
	${PROJECT_SOURCE_DIR}/gmapping/
	${PROJECT_SOURCE_DIR}/karto/
	${PROJECT_SOURCE_DIR}/amcl/
	${SuiteSparse_INCLUDE_DIRS}
)

file(GLOB MAP_TEST_SRC "*.cpp")
add_compile_options(-g)

add_subdirectory(common)
add_subdirectory(amcl)

add_executable(agv_main ${MAP_TEST_SRC})

target_link_libraries(agv_main AGVCommon AMCLCommon)
target_link_libraries(agv_main -ljsoncpp )

target_link_libraries(agv_main ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer -lboost_system -lboost_thread)
add_dependencies(agv_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(agv_main PUBLIC ${INCLUDE_CARTO})
