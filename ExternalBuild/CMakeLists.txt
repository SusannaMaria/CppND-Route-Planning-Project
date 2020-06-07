cmake_minimum_required(VERSION 3.11.3)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)

# Add the path of the cmake files to the CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

project(OSM_A_star_search)

# Set library output path to /lib
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")


include(ExternalProject)

ExternalProject_Add(
    io2d
  GIT_REPOSITORY https://github.com/cpp-io2d/P0267_RefImpl.git
  GIT_TAG        master
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/io2d
)
set(_FINDPACKAGE_IO2D_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/io2d/lib/cmake/io2d") 

ExternalProject_Add(
  OSM_A_star_search
  PREFIX osm_astar
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.."
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/osm_astar"  
  INSTALL_COMMAND ""
  CMAKE_ARGS -Dio2d_DIR:PATH=${_FINDPACKAGE_IO2D_CONFIG_DIR}
  DEPENDS io2d
)

add_custom_command(TARGET OSM_A_star_search POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/osm_astar/OSM_A_star_search ${CMAKE_CURRENT_BINARY_DIR}
)                       