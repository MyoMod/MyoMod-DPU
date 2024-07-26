cmake_minimum_required(VERSION 3.1...3.27)

option(BUILD_SHARED_LIBS "Build using object libraries" 0)

# turn variable on for verbose output, useful for build debugging. Or run 'ninja --verbose' in the build folder
#set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "ON")
# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

# Setup project name
project(DPU_CMake)

# set variables for directories
set(PROJECT_ROOT_DIR  "${CMAKE_CURRENT_SOURCE_DIR}")
set(EXECUTABLE         ${PROJECT_NAME}.elf)
set(BUILD_DIR         "${PROJECT_ROOT_DIR}/build")

set(LINKER_FILE "DPU_Debug.ld")

enable_language(C ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Optional: issue a message to be sure it uses the correct toolchain file.
message(STATUS "CMAKE_TOOLCHAIN_FILE is: ${CMAKE_TOOLCHAIN_FILE}")

# List of source files
set(SRC_FILES
  xip/evkcmimxrt1060_flexspi_nor_config.c
  xip/fsl_flexspi_nor_boot.c
  source/RuntimeHandler.cpp
  startup/startup_mimxrt1062.cpp
  device/system_MIMXRT1062.c
)

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SRC_FILES})

# Defines for the project
set(PROJECT_DEFINES
  -DCPU_MIMXRT1062DVL6B
  -DCCPU_MIMXRT1062DVL6B_cm7
  -DSDK_OS_BAREMETAL
  -DDISABLEFLOAT16
  -DSERIAL_PORT_TYPE_UART=1
  -DXIP_EXTERNAL_FLASH=1
  -DXIP_BOOT_HEADER_ENABLE=1
  -DSDK_DEBUGCONSOLE=1
  -D__MCUXPRESSO
  -D__USE_CMSIS
  -DDEBUG
  -D__NEWLIB__
  -DCR_INTEGER_PRINTF
  -DPRINTF_FLOAT_ENABLE=0
  -DSERIAL_PORT_TYPE_UART=1
)

# List of compiler defines, prefix with -D compiler option. Copy from the project settings
add_compile_options(
  ${PROJECT_DEFINES}
  -O0 
  -fno-common 
  -g3 
  -Wall 

  -ffunction-sections 
  -fdata-sections

  #remove ffreestanding and fno-building for gcc 13, as newlib acts like a hosted environment; 
  #see https://www.reddit.com/r/embedded/comments/17rx6ud/psa_gcc_13_with_ffreestanding_breaks_c_builds/
  #-ffreestanding
  #-fno-builtin
  -fno-exceptions

  #-fmerge-constants check if this is needed
  -fstack-usage
  -mcpu=cortex-m7
  -mfpu=fpv5-sp-d16
  -mfloat-abi=hard 
  -mthumb
  -specs=nano.specs
)

target_compile_definitions(${EXECUTABLE} PRIVATE
  ${PROJECT_DEFINES}
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
  -L "${CMAKE_SOURCE_DIR}/ld"
  -T "${LINKER_FILE}"
)

# needed for ETL lib
set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)

# add linker flags to print cross-reference table in map file and memory usage on console
add_link_options(-Wl,--cref,--print-memory-usage)
add_subdirectory(./etl              etl)
add_subdirectory(./board            board)
add_subdirectory(./CMSIS/DSP/Source CMSISDSP)
add_subdirectory(./drivers          drivers)
add_subdirectory(./utilities        utilities)
add_subdirectory(./component        component)
add_subdirectory(./source           source)
#add_subdirectory(./xip              xip)
add_subdirectory(./SEGGER_RTT       segger_rtt)

target_link_libraries(
    ${EXECUTABLE}
    PUBLIC board
    PUBLIC drivers
    PUBLIC utilities
    PUBLIC component
    PUBLIC source
#   PUBLIC xip
    PUBLIC segger_rtt
    PUBLIC CMSISDSP
)

target_include_directories(
    ${EXECUTABLE}
    PUBLIC
    ./CMSIS
)

# Optional: Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE}
)

# Optional: Create hex, bin and S-Record files after the build
add_custom_command(TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.s19"
  COMMAND ${CMAKE_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
  COMMAND ${CMAKE_OBJCOPY} -O binary "${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin" 
)