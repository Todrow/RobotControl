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

# ---------------------------------------------------------------------------
# LDROBOT LiDAR SDK (STL-19P handled as LD_19).
# Clone into mock_robot/ldlidar_sdk:
#   git clone https://github.com/ldrobotSensorTeam/ldlidar_sdk
# Built as a separate library so the vendor code is exempt from our strict
# warning flags. Optional: only compiled when the SDK folder is present.
# ---------------------------------------------------------------------------
set(LDLIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_sdk")
set(HAVE_LIDAR_SDK FALSE)
if(EXISTS "${LDLIDAR_SDK_DIR}/include/ldlidar_driver/ldlidar_driver.h")
    set(HAVE_LIDAR_SDK TRUE)
    if(WIN32)
        set(LDLIDAR_PLATFORM_SRC
            "${LDLIDAR_SDK_DIR}/src/ldlidar_driver_win.cpp"
            "${LDLIDAR_SDK_DIR}/src/serial_interface_win.cpp")
    else()
        set(LDLIDAR_PLATFORM_SRC
            "${LDLIDAR_SDK_DIR}/src/ldlidar_driver_linux.cpp"
            "${LDLIDAR_SDK_DIR}/src/network_socket_interface_linux.cpp"
            "${LDLIDAR_SDK_DIR}/src/serial_interface_linux.cpp")
    endif()
    add_library(ldlidar_sdk STATIC
        "${LDLIDAR_SDK_DIR}/src/ldlidar_driver.cpp"
        "${LDLIDAR_SDK_DIR}/src/ldlidar_dataprocess.cpp"
        "${LDLIDAR_SDK_DIR}/src/ldlidar_protocol.cpp"
        "${LDLIDAR_SDK_DIR}/src/log_module.cpp"
        "${LDLIDAR_SDK_DIR}/src/sl_transform.cpp"
        "${LDLIDAR_SDK_DIR}/src/slbf.cpp"
        "${LDLIDAR_SDK_DIR}/src/tofbf.cpp"
        ${LDLIDAR_PLATFORM_SRC})
    target_include_directories(ldlidar_sdk PUBLIC "${LDLIDAR_SDK_DIR}/include")
    target_link_libraries(ldlidar_sdk PUBLIC Threads::Threads)
    if(WIN32)
        target_compile_definitions(ldlidar_sdk PRIVATE NOMINMAX)
        target_link_libraries(ldlidar_sdk PUBLIC setupapi cfgmgr32)
    endif()
    # Vendor code: do not apply our strict warnings.
    if(MSVC)
        target_compile_options(ldlidar_sdk PRIVATE /W0)
    else()
        target_compile_options(ldlidar_sdk PRIVATE -w)
    endif()
else()
    message(WARNING
        "ldlidar_sdk not found at ${LDLIDAR_SDK_DIR}. Building WITHOUT the LiDAR "
        "guard: --lidar will report a start failure and the robot runs unguarded. "
        "Clone the SDK there to enable it.")
endif()

add_executable(mock_robot
    main.cpp
    command_listener.cpp
    telemetry_sender.cpp
    video_sender.cpp
    drive_controller.cpp
    servo_controller.cpp
    lidar_guard.cpp
)
target_include_directories(mock_robot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(mock_robot PRIVATE Threads::Threads gstreamer)

if(HAVE_LIDAR_SDK)
    target_link_libraries(mock_robot PRIVATE ldlidar_sdk)
    target_compile_definitions(mock_robot PRIVATE HAVE_LIDAR_SDK=1)
endif()

if(WIN32)
    target_compile_definitions(mock_robot PRIVATE NOMINMAX)
    target_link_libraries(mock_robot PRIVATE ws2_32)
endif()
if(MSVC)
    target_compile_options(mock_robot PRIVATE /W4)
else()
    target_compile_options(mock_robot PRIVATE -Wall -Wextra -Wpedantic)
endif()
