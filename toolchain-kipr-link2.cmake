# A CMake toolchain file for cross-compiling to the Link2 (STM...TODO) (Cortex-M4)

# usage (from wallaby dir):
# mkdir -p build && cd $_
# cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-kipr-link2.cmake ../

# We're going to cross-compile.  Need the macros to force which compiler to use.
include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME           Generic)
set(CMAKE_SYSTEM_PROCESSOR      cortext-m4)

CMAKE_FORCE_C_COMPILER(  arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

# TODO: This works only on a machine dedicated to cross-compiling
# see https://github.com/swift-nav/libswiftnav/blob/master/cmake/Toolchain-gcc-arm-embedded.cmake
# for how to find and set CMAKE_INSTALL_PREFIX and CMAKE_FIND_ROOT_PATH

# TODO set CXX and all the other flags. see
# http://stackoverflow.com/questions/16588097/cmake-separate-linker-and-compiler-flags



# http://www.valvers.com/open-software/raspberry-pi/step03-bare-metal-programming-in-c-pt3/
