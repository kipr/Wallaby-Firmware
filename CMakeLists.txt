
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("wallaby")

include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/STM32F4xx_StdPeriph_Driver/inc)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Include)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include)

add_definitions(-DSTM32F427_437xx)
add_definitions(-DUSE_STDPERIPH_DRIVER)

include_directories(stm32)
add_subdirectory(stm32)

file(GLOB wallaby_SRC "*.h" "*.c" "*.cpp")

add_executable(wallaby ${wallaby_SRC})
target_link_libraries(wallaby stm32)

