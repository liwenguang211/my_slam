
find_package(cartographer REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)

file(GLOB COMMON_SRC "*.cpp")
add_compile_options(-g)

add_library (AGVCommon ${COMMON_SRC})

target_link_libraries(AGVCommon ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)
add_dependencies(AGVCommon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(AGVCommon PUBLIC ${INCLUDE_CARTO})
