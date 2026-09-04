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
    command_listener.cpp
    telemetry_sender.cpp
    video_sender.cpp
    drive_controller.cpp
    servo_controller.cpp
)
target_include_directories(mock_robot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")
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
