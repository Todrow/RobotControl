cmake_minimum_required(VERSION 3.21)

# Also allow: cmake -S mock_robot -B build-pi
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(mock_robot LANGUAGES CXX)
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
    robot_find_gstreamer(FALSE)
endif()

add_executable(mock_robot
    main.cpp
    system_telemetry.cpp
    net/command_listener.cpp
    net/telemetry_sender.cpp
    net/video_sender.cpp
    actuators/drive_controller.cpp
    actuators/servo_controller.cpp
)
# mock_robot/ on the include path: sources say "net/...", "actuators/...",
# "util/..." instead of counting ../ hops. tests/test_drive_uart.py passes the
# same -I, so the harness compiles the same sources unchanged.
target_include_directories(mock_robot PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(mock_robot PRIVATE Threads::Threads gstreamer)
if(WIN32)
    target_compile_definitions(mock_robot PRIVATE NOMINMAX)
    target_link_libraries(mock_robot PRIVATE ws2_32)
endif()
if(MSVC)
    target_compile_options(mock_robot PRIVATE /W4)
else()
    target_compile_options(mock_robot PRIVATE -Wall -Wextra -Wpedantic)
endif()
