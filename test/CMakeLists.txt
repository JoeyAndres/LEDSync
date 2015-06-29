file(GLOB MUSIC_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*.ogg")

foreach(loop_var ${MUSIC_FILES})
    configure_file("${loop_var}" "${loop_var}" COPYONLY)
endforeach()

add_executable(play "main.cpp")
target_link_libraries(play LEDSync sfml-audio sfml-system)