# https://stackoverflow.com/questions/71365643/how-to-properly-use-cmake-in-embedded-rp2040-project-structure
# file mask CMakeLists.txt find text in file INTERFACE)

project(sysview)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# note cmake policy is to use absolute paths for interface libraries.
target_sources(${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT.c
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER_RTT_printf.c
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER_SYSVIEW_Config_NoOS_CM0.c
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER_SYSVIEW.c
)

target_link_libraries(${PROJECT_NAME} INTERFACE hardware_exception)