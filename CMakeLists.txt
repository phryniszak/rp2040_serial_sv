# https://forums.raspberrypi.com/viewtopic.php?t=331421
cmake_minimum_required(VERSION 3.12)

# set(tools /home/pawel/RP2040/gcc-arm-none-eabi-10.3-2021.10/)
# set(CMAKE_C_COMPILER ${tools}/bin/arm-none-eabi-gcc)
# set(CMAKE_CXX_COMPILER ${tools}/bin/arm-none-eabi-g++)
# set(CMAKE_ASM_COMPILER ${tools}/bin/arm-none-eabi-as)

# set compiler path
set(PICO_TOOLCHAIN_PATH /home/pawel/RP2040/gcc-arm-none-eabi-10.3-2021.10/bin)

# set SDK path
set(PICO_SDK_PATH ../../../pico-sdk)

# set board
set(PICO_BOARD pico)

# Pull in SDK (must be before project)
include(../pico_sdk_import.cmake)

project(blink C CXX ASM)

# You need to set the flags after the project command in your CMakeLists.txt.
# https://stackoverflow.com/questions/10085945/set-cflags-and-cxxflags-options-using-cmake
# set(CMAKE_BUILD_TYPE Debug)
# set(PICO_DEOPTIMIZED_DEBUG 1)
##### set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
# set(CMAKE_C_FLAGS_DEBUG "-ggdb")
# set(CMAKE_CXX_FLAGS_DEBUG "-ggdb")

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# add_compile_options(-Wall
#         -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#         -Wno-unused-function # we have some for the docs that aren't called
#         -Wno-maybe-uninitialized
#         )

# we need to remove the default UART driver since we replace it with our own, and having it
# in place will cause corruption
set(PICO_STDIO_UART 0)

add_subdirectory(sysview)

add_executable(blink
        src/main.c
)

# pull in common dependencies
target_link_libraries(blink pico_stdlib sysview)

# create map/bin/hex file etc.
# pico_add_extra_outputs(blink)

# Write to flash
# https://github.com/hikob/openlab/blob/master/platform/include-openocd.cmake
# add_custom_command(TARGET aaa.elf
#         POST_BUILD
#         COMMENT "write to flash ..."
#         COMMAND openocd -f ../flash.cfg
#         )