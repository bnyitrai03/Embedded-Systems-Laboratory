cmake_minimum_required(VERSION 3.16)

project(jiwy_controller C)

add_executable(jiwy_controller
    main.c
    control_loop.c
    control_protocol.c
    jiwy_calibration.c
    twentysim_controller.c
    homing.c
    rpi_spi_master.c
    controller/yaw_controller/yaw_model.c
    controller/yaw_controller/yaw_integ.c
    controller/yaw_controller/yaw_funcs.c
    controller/yaw_controller/yaw_submod.c
    controller/pitch_controller/pitch_model.c
    controller/pitch_controller/pitch_integ.c
    controller/pitch_controller/pitch_funcs.c
    controller/pitch_controller/pitch_submod.c
)

target_include_directories(jiwy_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/yaw_controller
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/pitch_controller
)

target_compile_features(jiwy_controller PRIVATE c_std_11)
target_compile_definitions(jiwy_controller PRIVATE
    _GNU_SOURCE
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
