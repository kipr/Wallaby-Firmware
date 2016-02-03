
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("wallaby")
#set(CMAKE_VERBOSE_MAKEFILE ON)
enable_language(C ASM)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4")                                   # Cortex-M4 CPU
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb")                                           # Thumb instructions
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtp=soft")                                         # Use 'soft' model for storage of the thread-local-storage pointer
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mlittle-endian")                                   # little endian
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard")                                  # hardware FPU
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=fpv4-sp-d16")                                 # FPU type

# TODO: Where should these be located on a standard build machine / workstation?
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/STM32F4xx_StdPeriph_Driver/inc)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Include)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include)

# C defines for the ST 'periph' library
add_definitions(-DSTM32F427_437xx)
add_definitions(-DUSE_STDPERIPH_DRIVER)

# Add the ST 'periph' library include files
include_directories(stm32)

# All of the source files (wallaby and ST 'periph' library)
file(GLOB SRC "*.h" "*.c" "*.cpp")
file(GLOB SRC "stm32/*.h" "stm32/*.c")

# The ST 'periph' library includes fmc and fsmc -- but the 427 uses fmc
list(REMOVE_ITEM SRC
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32/stm32f4xx_fsmc.c"
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32/stm32f4xx_fsmc.h"
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32/stm32f4xx_pwr.c"
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32/stm32f4xx_pwr.h")

# Include the startup ASM
list(APPEND SRC "${CMAKE_CURRENT_SOURCE_DIR}/stm32/startup_stm32f427xx.s")

# The linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F437IIHx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}")

add_executable(wallaby ${SRC})

