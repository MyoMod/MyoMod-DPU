file(GLOB FILES
*.c
algorithms/*.c
commonClasses/*.c
communication/*.c
configurations/*.c
devices/*.c
#structure/*.c
*.cpp
algorithms/*.cpp
commonClasses/*.cpp
communication/*.cpp
configurations/*.cpp
devices/*.cpp
#structure/*.cpp
 )
 
# add_library: With this declaration, you express the intent to build a library. 
# The first argument, here its pico-shift-register, is the name of the library, 
# the second argument are the files that will be compiled to create your library.
add_library(source ${FILES})
 
# target_link_libraries: If you link with other libraries, list them here
target_link_libraries(
  source 
  drivers
  segger_rtt
  CMSISDSP
  component
  utilities
)
 
# target_include_directories: Libraries need to publish their header files 
# so that you can import them in source code. This statement expresses where to find the files 
# - typically in an include directory of your projects.
target_include_directories(
  source 
  PUBLIC
  ./
  ./algorithms
  ./commonClasses
  ./communication
  ./configurations
  ./devices
  ./structure
  ../CMSIS
  ../CMSIS/DSP/Include
)