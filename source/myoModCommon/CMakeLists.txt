file(GLOB FILES
${CMAKE_CURRENT_SOURCE_DIR}/*.c
 )
 
# add_library: With this declaration, you express the intent to build a library. 
# The first argument, here its pico-shift-register, is the name of the library, 
# the second argument are the files that will be compiled to create your library.
add_library(myoModCommon ${FILES})
add_library(myoModCommon::myoModCommon ALIAS myoModCommon)

# target_link_libraries: If you link with other libraries, list them here
target_link_libraries(
  myoModCommon PUBLIC
  drivers
  board
)
 
# target_include_directories: Libraries need to publish their header files 
# so that you can import them in source code. This statement expresses where to find the files 
# - typically in an include directory of your projects.
target_include_directories(
  myoModCommon 
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)