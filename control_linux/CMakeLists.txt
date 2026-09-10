cmake_minimum_required(VERSION 3.21)

# Linux port of control/. Build it from the repo root with
#   cmake -S . -B build-linux -DBUILD_CONTROL_LINUX=ON -DBUILD_MOCK_ROBOT=OFF
#   cmake --build build-linux -j
# or standalone straight from this directory:
#   cmake -S control_linux -B build-control -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-control -j
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(control_linux LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER "${cfg}" cfg_upper)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg_upper} "${CMAKE_BINARY_DIR}")
    endforeach()
    find_package(Threads REQUIRED)
    include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/GStreamer.cmake")
    robot_find_gstreamer(TRUE)  # the client needs gstreamer-video-1.0 for GstVideoOverlay
endif()

if(WIN32)
    message(FATAL_ERROR "control_linux is the POSIX port. On Windows build the control target instead.")
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(control_linux
    main.cpp
    net/tcp_connection.cpp
    net/connection_manager.cpp
    net/command_sender.cpp
    net/telemetry_receiver.cpp
    state/desired_state_slot.cpp
    input/input_controller.cpp
    video/video_receiver.cpp
    ui/main_window.cpp
    ui/hud_overlay.cpp
    ui/yaw_indicator.cpp
)
set_target_properties(control_linux PROPERTIES AUTOMOC ON)

# control_linux/ and common/ on the include path so sources say "net/...",
# "ui/...", "../../common/protocol.h" exactly as the Windows target does.
target_include_directories(control_linux PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(control_linux PRIVATE Qt6::Widgets gstreamer Threads::Threads)
target_compile_options(control_linux PRIVATE -Wall -Wextra)
