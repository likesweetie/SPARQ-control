cmake_minimum_required(VERSION 3.16)
project(SPARQ CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRD_PARTY_DIR ${PROJECT_ROOT}/third_party)
set(MUJOCO_DIR ${THIRD_PARTY_DIR}/mujoco)
set(ONNX_DIR ${THIRD_PARTY_DIR}/onnxruntime)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)


add_executable(mujoco_simulate
    ${MUJOCO_DIR}/simulate/main.cc
    ${MUJOCO_DIR}/simulate/glfw_adapter.cc
    ${MUJOCO_DIR}/simulate/glfw_dispatch.cc
    ${MUJOCO_DIR}/simulate/platform_ui_adapter.cc
    ${MUJOCO_DIR}/simulate/simulate.cc
    ${MUJOCO_DIR}/simulate/lodepng.cpp
    ${MUJOCO_DIR}/simulate/mujoco_can/mujoco_can_bridge.cc
    ${MUJOCO_DIR}/simulate/mujoco_can/socket_can_adapter.cc
    ${MUJOCO_DIR}/simulate/mujoco_can/actuator_firmware_base.cc
    ${MUJOCO_DIR}/simulate/mujoco_can/spg_firmware.cc
    ${MUJOCO_DIR}/simulate/mujoco_can/e2box_imu_firmware.cc
)
target_include_directories(mujoco_simulate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_DIR}/include
    ${MUJOCO_DIR}/simulate
    ${MUJOCO_DIR}/simulate/mujoco_can
    ${GLFW_INCLUDE_DIRS}
)
target_link_directories(mujoco_simulate PRIVATE
    ${MUJOCO_DIR}/lib
)
target_link_libraries(mujoco_simulate PRIVATE
    ${GLFW_LIBRARIES}
    mujoco
    yaml-cpp
    GL
    pthread
    dl
    m
)
set_target_properties(mujoco_simulate PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib"
)
target_compile_definitions(mujoco_simulate PRIVATE
    MUJOCO_SIM_CONFIG_DIR="${MUJOCO_DIR}/simulate/configs"
)
