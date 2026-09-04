# Locates GStreamer and exposes it as the INTERFACE target `gstreamer`.
#
#   robot_find_gstreamer(<need_video>)
#
# <need_video> is true for the Qt control client, which additionally uses
# gstreamer-video-1.0 for GstVideoOverlay. The robot server only needs the core
# library. pkg-config is preferred; a manual search covers the Windows MSVC
# installer, which ships no .pc files.

include_guard(GLOBAL)

function(robot_find_gstreamer need_video)
    if(TARGET gstreamer)
        return()
    endif()

    set(modules gstreamer-1.0)
    if(need_video)
        list(APPEND modules gstreamer-video-1.0)
    endif()

    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(ROBOT_GST QUIET IMPORTED_TARGET GLOBAL ${modules})
    endif()

    if(ROBOT_GST_FOUND)
        add_library(gstreamer INTERFACE)
        target_link_libraries(gstreamer INTERFACE PkgConfig::ROBOT_GST)
        message(STATUS "GStreamer ${ROBOT_GST_gstreamer-1.0_VERSION} via pkg-config")
        return()
    endif()

    set(roots)
    if(GSTREAMER_ROOT)
        list(APPEND roots "${GSTREAMER_ROOT}")
    endif()
    foreach(var GSTREAMER_1_0_ROOT_MSVC_X86_64 GSTREAMER_1_0_ROOT_MINGW_X86_64 GSTREAMER_1_0_ROOT_X86_64)
        if(DEFINED ENV{${var}})
            list(APPEND roots "$ENV{${var}}")
        endif()
    endforeach()
    list(APPEND roots "C:/gstreamer/1.0/msvc_x86_64" "C:/gstreamer/1.0/mingw_x86_64")

    # gstconfig.h and glibconfig.h are generated headers and live under lib/.
    find_path(GSTREAMER_INCLUDE_DIR gst/gst.h
        PATHS ${roots} PATH_SUFFIXES include/gstreamer-1.0)
    find_path(GSTREAMER_CONFIG_INCLUDE_DIR gst/gstconfig.h
        PATHS ${roots} PATH_SUFFIXES lib/gstreamer-1.0/include include/gstreamer-1.0)
    find_path(GLIB_INCLUDE_DIR glib.h
        PATHS ${roots} PATH_SUFFIXES include/glib-2.0)
    find_path(GLIB_CONFIG_INCLUDE_DIR glibconfig.h
        PATHS ${roots} PATH_SUFFIXES lib/glib-2.0/include include/glib-2.0)

    find_library(GSTREAMER_LIBRARY NAMES gstreamer-1.0 PATHS ${roots} PATH_SUFFIXES lib)
    find_library(GOBJECT_LIBRARY NAMES gobject-2.0 PATHS ${roots} PATH_SUFFIXES lib)
    find_library(GLIB_LIBRARY NAMES glib-2.0 PATHS ${roots} PATH_SUFFIXES lib)

    set(required
        GSTREAMER_INCLUDE_DIR GSTREAMER_CONFIG_INCLUDE_DIR
        GLIB_INCLUDE_DIR GLIB_CONFIG_INCLUDE_DIR
        GSTREAMER_LIBRARY GOBJECT_LIBRARY GLIB_LIBRARY)
    set(libs "${GSTREAMER_LIBRARY}" "${GOBJECT_LIBRARY}" "${GLIB_LIBRARY}")

    if(need_video)
        find_library(GSTREAMER_VIDEO_LIBRARY NAMES gstvideo-1.0 PATHS ${roots} PATH_SUFFIXES lib)
        list(APPEND required GSTREAMER_VIDEO_LIBRARY)
        list(APPEND libs "${GSTREAMER_VIDEO_LIBRARY}")
    endif()

    foreach(var ${required})
        if(NOT ${var})
            message(FATAL_ERROR
                "GStreamer development files not found (${var} is missing).\n"
                "Linux: install libgstreamer1.0-dev (and libgstreamer-plugins-base1.0-dev for the client).\n"
                "Windows: install the GStreamer runtime + development packages and pass "
                "-DGSTREAMER_ROOT=C:/gstreamer/1.0/msvc_x86_64.")
        endif()
    endforeach()

    add_library(gstreamer INTERFACE)
    target_include_directories(gstreamer SYSTEM INTERFACE
        "${GSTREAMER_INCLUDE_DIR}" "${GSTREAMER_CONFIG_INCLUDE_DIR}"
        "${GLIB_INCLUDE_DIR}" "${GLIB_CONFIG_INCLUDE_DIR}")
    target_link_libraries(gstreamer INTERFACE ${libs})
    message(STATUS "GStreamer found at ${GSTREAMER_LIBRARY}")
endfunction()
