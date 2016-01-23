
file(GLOB stm32_SRC "*.h" "*.c")
list(REMOVE_ITEM stm32_SRC 
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_fsmc.c" 
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_fsmc.h" 
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_pwr.c" 
      "${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_pwr.h")

include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/STM32F4xx_StdPeriph_Driver/inc)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Include)
include_directories(~/dev/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include)

add_definitions(-DSTM32F427_437xx)
add_definitions(-DUSE_STDPERIPH_DRIVER)

add_library(stm32 ${stm32_SRC})

