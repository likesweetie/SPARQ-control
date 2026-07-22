cmake_minimum_required(VERSION 3.16)

project(imu_serial LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mip REQUIRED CONFIG)

find_package(
    microstrain
    REQUIRED
    COMPONENTS microstrain_serial
    CONFIG
)

add_executable(
    imu_serial
    main.cpp
)

target_compile_definitions(
    imu_serial
    PRIVATE
        MICROSTRAIN_USE_STD_ENDIAN=1
)

target_include_directories(
    imu_serial
    PRIVATE
        ${MIP_INCLUDE_DIRS}
        ${MICROSTRAIN_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    imu_serial
    PRIVATE
        mip
        microstrain_serial
        microstrain
        pthread
        rt
)

target_compile_options(
    imu_serial
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)