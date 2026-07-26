cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# MercuryBSP — reusable STM32 BSP + common Modules
#
# This file does NOT create a standalone firmware target. It exposes two
# helper functions that inject sources/includes into YOUR CubeMX target:
#
#   mercury_add_bsp(<target>
#       [COMPONENTS dwt gpio log iic can usart spi pwm usb segger]
#       [ALL])
#
#   mercury_add_modules(<target>
#       [COMPONENTS Algorithm MessageCenter JScope Daemon LED Buzzer
#                   Motor IMU Referee Vision]
#       [ALL])
#
# The consumer target must already have HAL/CMSIS/FreeRTOS include paths
# (CubeMX stm32cubemx link or equivalent) before or after these calls.
# ---------------------------------------------------------------------------

if(NOT DEFINED MERCURY_BSP_ROOT)
    set(MERCURY_BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "MercuryBSP root")
endif()

set(_MERCURY_BSP_COMPONENTS dwt gpio log iic can usart spi pwm usb segger)
set(_MERCURY_MOD_COMPONENTS Algorithm MessageCenter JScope Daemon LED Buzzer Motor IMU Referee Vision)

# Map component name -> relative source globs / include dirs / compile defs
function(_mercury_bsp_component_files comp out_sources out_includes out_defs)
    set(_src "")
    set(_inc "")
    set(_def "")
    if(comp STREQUAL "dwt")
        set(_src "${MERCURY_BSP_ROOT}/BSP/dwt/bsp_dwt.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/dwt")
    elseif(comp STREQUAL "gpio")
        set(_src "${MERCURY_BSP_ROOT}/BSP/gpio/bsp_gpio.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/gpio")
    elseif(comp STREQUAL "log")
        set(_src "${MERCURY_BSP_ROOT}/BSP/log/bsp_log.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/log")
    elseif(comp STREQUAL "iic")
        set(_src
            "${MERCURY_BSP_ROOT}/BSP/iic/bsp_iic.cpp"
            "${MERCURY_BSP_ROOT}/BSP/iic/bsp_iic_slave.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/iic")
        set(_def "MERCURY_BSP_HAS_IIC=1")
    elseif(comp STREQUAL "can")
        set(_src "${MERCURY_BSP_ROOT}/BSP/can/bsp_can.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/can")
        set(_def "MERCURY_BSP_HAS_CAN=1")
    elseif(comp STREQUAL "usart")
        set(_src "${MERCURY_BSP_ROOT}/BSP/usart/bsp_usart.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/usart")
        set(_def "MERCURY_BSP_HAS_USART=1")
    elseif(comp STREQUAL "spi")
        set(_src "${MERCURY_BSP_ROOT}/BSP/spi/bsp_spi.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/spi")
        set(_def "MERCURY_BSP_HAS_SPI=1")
    elseif(comp STREQUAL "pwm")
        set(_src "${MERCURY_BSP_ROOT}/BSP/pwm/bsp_pwm.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/pwm")
        set(_def "MERCURY_BSP_HAS_PWM=1")
    elseif(comp STREQUAL "usb")
        set(_src "${MERCURY_BSP_ROOT}/BSP/usb/bsp_usb.cpp")
        set(_inc "${MERCURY_BSP_ROOT}/BSP/usb")
        set(_def "MERCURY_BSP_HAS_USB=1")
    elseif(comp STREQUAL "segger")
        set(_src
            "${MERCURY_BSP_ROOT}/third_party/SEGGER/SEGGER_RTT.c"
            "${MERCURY_BSP_ROOT}/third_party/SEGGER/SEGGER_RTT_printf.c")
        # ASM optional — enable with MERCURY_BSP_SEGGER_ASM=ON on Cortex-M3/M4/M7
        if(MERCURY_BSP_SEGGER_ASM)
            list(APPEND _src
                "${MERCURY_BSP_ROOT}/third_party/SEGGER/SEGGER_RTT_ASM_ARMv7M.S")
        endif()
        set(_inc "${MERCURY_BSP_ROOT}/third_party/SEGGER")
    else()
        message(FATAL_ERROR "MercuryBSP: unknown BSP component '${comp}'")
    endif()
    set(${out_sources} "${_src}" PARENT_SCOPE)
    set(${out_includes} "${_inc}" PARENT_SCOPE)
    set(${out_defs} "${_def}" PARENT_SCOPE)
endfunction()

function(_mercury_mod_component_files comp out_sources out_includes)
    set(_src "")
    set(_inc "${MERCURY_BSP_ROOT}/Modules/${comp}")
    if(comp STREQUAL "Algorithm")
        set(_src
            "${MERCURY_BSP_ROOT}/Modules/Algorithm/controller.cpp"
            "${MERCURY_BSP_ROOT}/Modules/Algorithm/ekf.cpp"
            "${MERCURY_BSP_ROOT}/Modules/Algorithm/user_lib.cpp")
    elseif(comp STREQUAL "MessageCenter")
        set(_src "${MERCURY_BSP_ROOT}/Modules/MessageCenter/message_center.cpp")
    elseif(comp STREQUAL "JScope")
        # header-only (+ .inl)
        set(_src "")
    elseif(comp STREQUAL "Daemon")
        set(_src "${MERCURY_BSP_ROOT}/Modules/Daemon/daemon.cpp")
    elseif(comp STREQUAL "LED")
        set(_src "${MERCURY_BSP_ROOT}/Modules/LED/led.cpp")
    elseif(comp STREQUAL "Buzzer")
        set(_src "${MERCURY_BSP_ROOT}/Modules/Buzzer/buzzer.cpp")
    elseif(comp STREQUAL "Motor")
        set(_src
            "${MERCURY_BSP_ROOT}/Modules/Motor/dji_motor.cpp"
            "${MERCURY_BSP_ROOT}/Modules/Motor/dm_motor.cpp")
    elseif(comp STREQUAL "IMU")
        set(_src
            "${MERCURY_BSP_ROOT}/Modules/IMU/imu.cpp"
            "${MERCURY_BSP_ROOT}/Modules/IMU/bmi088.cpp"
            "${MERCURY_BSP_ROOT}/Modules/IMU/canopen_parser.c"
            "${MERCURY_BSP_ROOT}/Modules/IMU/hipnuc_can_common.c")
    elseif(comp STREQUAL "Referee")
        set(_src
            "${MERCURY_BSP_ROOT}/Modules/Referee/referee.cpp"
            "${MERCURY_BSP_ROOT}/Modules/Referee/referee_ui.cpp")
    elseif(comp STREQUAL "Vision")
        set(_src "${MERCURY_BSP_ROOT}/Modules/Vision/vision.cpp")
    else()
        message(FATAL_ERROR "MercuryBSP: unknown Modules component '${comp}'")
    endif()
    set(${out_sources} "${_src}" PARENT_SCOPE)
    set(${out_includes} "${_inc}" PARENT_SCOPE)
endfunction()

function(mercury_add_bsp TARGET)
    cmake_parse_arguments(ARG "ALL" "" "COMPONENTS" ${ARGN})
    if(ARG_ALL OR NOT ARG_COMPONENTS)
        set(ARG_COMPONENTS ${_MERCURY_BSP_COMPONENTS})
    endif()

    # Always expose BSP/ for umbrella bsp.h
    target_include_directories(${TARGET} PRIVATE
        "${MERCURY_BSP_ROOT}/BSP"
    )

    foreach(comp IN LISTS ARG_COMPONENTS)
        _mercury_bsp_component_files(${comp} _src _inc _def)
        if(_src)
            target_sources(${TARGET} PRIVATE ${_src})
        endif()
        if(_inc)
            target_include_directories(${TARGET} PRIVATE ${_inc})
        endif()
        if(_def)
            target_compile_definitions(${TARGET} PRIVATE ${_def})
        endif()
    endforeach()
endfunction()

function(mercury_add_modules TARGET)
    cmake_parse_arguments(ARG "ALL" "" "COMPONENTS" ${ARGN})
    if(ARG_ALL OR NOT ARG_COMPONENTS)
        set(ARG_COMPONENTS ${_MERCURY_MOD_COMPONENTS})
    endif()

    target_include_directories(${TARGET} PRIVATE
        "${MERCURY_BSP_ROOT}/Modules"
    )

    foreach(comp IN LISTS ARG_COMPONENTS)
        _mercury_mod_component_files(${comp} _src _inc)
        if(_src)
            target_sources(${TARGET} PRIVATE ${_src})
        endif()
        if(_inc)
            target_include_directories(${TARGET} PRIVATE ${_inc})
        endif()
    endforeach()
endfunction()

# Convenience: both layers with sensible defaults for a new project
function(mercury_add_all TARGET)
    mercury_add_bsp(${TARGET} ALL)
    mercury_add_modules(${TARGET} ALL)
endfunction()
