cmake_minimum_required(VERSION 3.16)

project(jiwy_controller C)

add_executable(jiwy_controller
    main.c
    control_loop.c
    control_protocol.c
    controller_adapter.c
    homing.c
    rpi_spi_master.c
    controller/pitch_controller.c
    controller/yaw_controller.c
    controller/xxfuncs.c
)

target_include_directories(jiwy_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(jiwy_controller PRIVATE c_std_11)
target_compile_definitions(jiwy_controller PRIVATE
    _POSIX_C_SOURCE=200809L
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(jiwy_controller PRIVATE
        -Wall
        -Wextra
    )
endif()

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(jiwy_controller PRIVATE ${MATH_LIBRARY})
endif()
