file(GLOB FILES
*.c
algorithms/*.c
communication/*.c
configurations/*.c
devices/*.c
#structure/*.c
hardware/*.c
hardware/*.cpp
*.cpp
algorithms/*.cpp
commonClasses/*.cpp
communication/*.cpp
configurations/*.cpp
devices/*.cpp
embeddedDevices/*.cpp
#structure/*.cpp
 )
 
# add_library: With this declaration, you express the intent to build a library. 
# The first argument, here its pico-shift-register, is the name of the library, 
# the second argument are the files that will be compiled to create your library.
add_library(source ${FILES})
 

add_subdirectory(./configurations/lwjson        lwjson)

add_subdirectory(./myoModCommon                 myoModCommon)
add_subdirectory(./embeddedDevices/icm42670p    icm42670p)
add_subdirectory(./embeddedDevices/max11254     max11254)

# target_link_libraries: If you link with other libraries, list them here
target_link_libraries(
  source 
  drivers
  segger_rtt
  CMSISDSP
  component
  utilities
  lwjson
  etl
  myoModCommon
  icm42670p
  max11254
)
 
# target_include_directories: Libraries need to publish their header files 
# so that you can import them in source code. This statement expresses where to find the files 
# - typically in an include directory of your projects.
target_include_directories(
  source 
  PUBLIC
  ./
  ./algorithms
  ./communication
  ./configurations
  ./devices
  ./embeddedDevices
  ./structure
  ./hardware
  ./generated
  ../CMSIS
  ../CMSIS/DSP/Include
  ../generated
)

# This function envelops the content of a file in a C++ raw string literal.
function(make_includable input_file output_file)
    file(READ ${input_file} content)
    set(delim "for_c++_include")
    set(content "R\"${delim}(\n${content})${delim}\"")
    file(WRITE ${output_file} "${content}")
endfunction(make_includable)

make_includable(../config.json generated/config.enveloped_json)