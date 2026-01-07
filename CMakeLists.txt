cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_compile_options(-mthumb -mcpu=cortex-m3 -mabi=aapcs -O3 -fdata-sections -ffunction-sections -mlong-calls)
add_link_options(-mthumb -mcpu=cortex-m3 -mabi=aapcs -O3 -fdata-sections -ffunction-sections -mlong-calls -Wl,-u,_printf_float)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32f103-micro-ros)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
# Add micro-ROS include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/Third_Party/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
    App/Main/Inc
    App/PID/Inc
    App/Motor/Inc
    App/IO/Led/Inc
    App/IO/Buzzer/Inc
    Drivers/BSP/motor/Inc
    Drivers/BSP/imu/icm20948/Inc
    Drivers/BSP/imu/ak09916/Inc
    Drivers/BSP/led/Inc
    Drivers/BSP/buzzer/Inc
    Drivers/BSP/iwdg/Inc
    App/Imu/Inc
    App/Watchdog/Inc
    App/IO/Key/Inc
    Drivers/BSP/key/Inc
    Drivers/BSP/microros/Inc
    App/Tasks/Inc
    App/SerialROS/Inc
    Drivers/BSP/serial_ros/Inc
)

# Add micro-ROS support source files
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Main/Src/global.c
    App/Motor/Src/motor.c
    App/PID/Src/pid.c
    App/IO/Led/Src/io_led.c
    App/IO/Buzzer/Src/io_buzzer.c
    Drivers/BSP/motor/Src/bsp_motor.c
    Drivers/BSP/imu/icm20948/Src/icm20948.c
    Drivers/BSP/imu/ak09916/Src/ak09916.c
    Drivers/BSP/led/Src/bsp_led.c
    Drivers/BSP/buzzer/Src/bsp_buzzer.c
    Drivers/BSP/iwdg/Src/bsp_iwdg.c
    App/Imu/Src/imu_app.c
    App/Watchdog/Src/wdg_app.c
    App/IO/Key/Src/io_key.c
    Drivers/BSP/key/Src/bsp_key.c
    App/Main/Src/task_hearthbeat.c
    App/Main/Src/task_controller.c
    App/Main/Src/task_manager.c
    App/Main/Src/task_imu.c
    App/Main/Src/task_encoder.c
    App/SerialROS/Src/serial_ros.c
    Drivers/BSP/serial_ros/Src/bsp_serial_ros.c
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# Ensure HEX and BIN files are cleaned on 'make clean'
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES
    "${CMAKE_PROJECT_NAME}.hex"
    "${CMAKE_PROJECT_NAME}.bin"
    "${CMAKE_PROJECT_NAME}.elf"
    "${CMAKE_PROJECT_NAME}.map"
)

# Remove old HEX and BIN files before building
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove -f "${CMAKE_PROJECT_NAME}.hex" "${CMAKE_PROJECT_NAME}.bin" "${CMAKE_PROJECT_NAME}.elf" "${CMAKE_PROJECT_NAME}.map"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Cleaning old HEX and BIN files"
)

# Generate .hex file
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating HEX file"
)

# Generate .bin file
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating BIN file"
)
