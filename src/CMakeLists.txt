file(GLOB SRC_FILES "*.cpp")

add_library(LEDSync ${SRC_FILES})
target_link_libraries(LEDSync boost_filesystem boost_system boost_iostreams)