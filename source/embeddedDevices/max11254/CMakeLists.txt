file(GLOB FILES
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
 )
 
# add_library: With this declaration, you express the intent to build a library. 
# The first argument, here its pico-shift-register, is the name of the library, 
# the second argument are the files that will be compiled to create your library.
add_library(max11254 ${FILES})
add_library(myomod::max11254 ALIAS max11254)
 
# target_link_libraries: If you link with other libraries, list them here
target_link_libraries(
  max11254 PRIVATE 
  drivers
  board
  myoModCommon
)
 
# target_include_directories: Libraries need to publish their header files 
# so that you can import them in source code. This statement expresses where to find the files 
# - typically in an include directory of your projects.
target_include_directories(
  max11254 
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/
)