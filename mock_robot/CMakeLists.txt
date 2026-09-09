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
    lidar/lidar_reader.cpp
    actuators/drive_controller.cpp
    actuators/servo_controller.cpp
)
# mock_robot/ on the include path: sources say "net/...", "actuators/...",
# "utils/..." instead of counting ../ hops. tests/test_drive_uart.py passes the
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

# ---------------------------------------------------------------------------
#  LDROBOT lidar SDK (third_party/ldlidar_sdk), built as a small static library
#  so its warnings and its C++ dialect stay out of the robot's own sources.
# ---------------------------------------------------------------------------
set(LDLIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ldlidar_sdk")
add_library(ldlidar_sdk STATIC
    "${LDLIDAR_SDK_DIR}/src/ldlidar_driver.cpp"
    "${LDLIDAR_SDK_DIR}/src/ldlidar_dataprocess.cpp"
    "${LDLIDAR_SDK_DIR}/src/ldlidar_protocol.cpp"
    "${LDLIDAR_SDK_DIR}/src/log_module.cpp"
    "${LDLIDAR_SDK_DIR}/src/sl_transform.cpp"
    "${LDLIDAR_SDK_DIR}/src/slbf.cpp"
    "${LDLIDAR_SDK_DIR}/src/tofbf.cpp"
)
if(WIN32)
    target_sources(ldlidar_sdk PRIVATE
        "${LDLIDAR_SDK_DIR}/src/ldlidar_driver_win.cpp"
        "${LDLIDAR_SDK_DIR}/src/serial_interface_win.cpp")
    target_link_libraries(ldlidar_sdk PUBLIC setupapi cfgmgr32)
else()
    target_sources(ldlidar_sdk PRIVATE
        "${LDLIDAR_SDK_DIR}/src/ldlidar_driver_linux.cpp"
        "${LDLIDAR_SDK_DIR}/src/serial_interface_linux.cpp"
        "${LDLIDAR_SDK_DIR}/src/network_socket_interface_linux.cpp")
endif()
# glibc pulls linux/sched/types.h from bits/sched.h, but that kernel header uses
# __u32/__s32 without including linux/types.h itself. When the two packages are
# out of step it breaks every C++ translation unit that reaches <pthread.h>, so
# force the missing header in front of both targets.
if(NOT WIN32)
    target_compile_options(ldlidar_sdk PRIVATE -include linux/types.h)
    target_compile_options(mock_robot PRIVATE -include linux/types.h)
endif()
target_include_directories(ldlidar_sdk PUBLIC "${LDLIDAR_SDK_DIR}/include")
target_link_libraries(ldlidar_sdk PUBLIC Threads::Threads)

target_link_libraries(mock_robot PRIVATE ldlidar_sdk)
