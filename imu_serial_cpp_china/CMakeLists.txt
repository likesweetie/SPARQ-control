cmake_minimum_required(VERSION 3.21)
project(imu_serial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(imu_serial main.cpp)

target_compile_options(imu_serial PRIVATE -Wall -Wextra)
target_link_libraries(imu_serial PRIVATE rt)
