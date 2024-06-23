cmake_minimum_required(VERSION 3.0.0)
project(testStructure VERSION 0.1.0 LANGUAGES C CXX)

add_executable(testStructure node.cpp test.cpp configParser.cpp)
set(LWJSON_OPTS_FILE ${CMAKE_CURRENT_LIST_DIR}/lwjson/lwjson_opts.h)

add_subdirectory(lwjson)
target_link_libraries(${PROJECT_NAME} lwjson)
