cmake_minimum_required(VERSION 2.8)
project(LEDSync)

# The version number.
set(GNUPlot++_VERSION_MAJOR 0)
set(GNUPlot++_VERSION_MINOR 1)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -fPIC")

include_directories(
  include)

add_subdirectory(src)
add_subdirectory(test)