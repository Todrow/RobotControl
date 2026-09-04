cmake_minimum_required(VERSION 3.21)
project(robot_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Raspberry Pi only needs the console server. The operator UI stays on Windows.
option(BUILD_CONTROL "Build the Windows Qt control application" ${WIN32})
option(BUILD_MOCK_ROBOT "Build the Raspberry Pi / Windows robot server" ON)
if(BUILD_CONTROL AND NOT WIN32)
    message(FATAL_ERROR "control is a Windows application. Use -DBUILD_CONTROL=OFF on Raspberry Pi.")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${cfg}" cfg_upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg_upper} "${CMAKE_BINARY_DIR}")
endforeach()

if(BUILD_CONTROL OR BUILD_MOCK_ROBOT)
    find_package(Threads REQUIRED)
    include(cmake/GStreamer.cmake)
    robot_find_gstreamer(${BUILD_CONTROL})
endif()

if(BUILD_CONTROL)
    find_package(Qt6 REQUIRED COMPONENTS Widgets)
    add_executable(control
        control/main.cpp
        control/net/tcp_connection.cpp
        control/net/connection_manager.cpp
        control/net/command_sender.cpp
        control/net/telemetry_receiver.cpp
        control/state/desired_state_slot.cpp
        control/input/input_controller.cpp
        control/video/video_receiver.cpp
        control/ui/main_window.cpp
        control/ui/hud_overlay.cpp
    )
    set_target_properties(control PROPERTIES AUTOMOC ON WIN32_EXECUTABLE TRUE)
    target_compile_definitions(control PRIVATE NOMINMAX)
    if(TARGET Qt6::EntryPointPrivate)
        target_link_libraries(control PRIVATE Qt6::EntryPointPrivate)
    endif()
    target_link_libraries(control PRIVATE Qt6::Widgets gstreamer Threads::Threads ws2_32)

    find_program(WINDEPLOYQT NAMES windeployqt windeployqt-qt6 HINTS "${Qt6_DIR}/../../../bin")
    if(WINDEPLOYQT)
        add_custom_command(TARGET control POST_BUILD
            COMMAND "${WINDEPLOYQT}" --no-translations --no-compiler-runtime
                    "$<TARGET_FILE:control>"
            COMMENT "windeployqt")
    endif()
endif()

if(BUILD_MOCK_ROBOT)
    # Keep CMake's subdirectory separate from the Linux executable mock_robot.
    add_subdirectory(mock_robot mock_robot-build)
endif()
