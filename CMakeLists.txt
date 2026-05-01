# Build configuration for the DroneNavReal C++ project.
cmake_minimum_required(VERSION 3.16)

project(DroneNavReal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SFML_COMPONENTS graphics window system)

# Change this path if SFML is installed elsewhere.
if(NOT SFML_DIR)
    set(SFML_DIR "C:/SFML/lib/cmake/SFML")
endif()

# Prefer an installed SFML package, including vcpkg toolchain integrations.
find_package(SFML 2.5 COMPONENTS ${SFML_COMPONENTS} CONFIG QUIET)

# Fallback for manual SFML installs on Windows.
if(NOT SFML_FOUND)
    set(SFML_ROOT "" CACHE PATH "Root directory of a manual SFML installation")

    set(_sfml_search_roots)
    if(SFML_ROOT)
        list(APPEND _sfml_search_roots "${SFML_ROOT}")
    endif()
    if(DEFINED ENV{SFML_ROOT} AND NOT "$ENV{SFML_ROOT}" STREQUAL "")
        list(APPEND _sfml_search_roots "$ENV{SFML_ROOT}")
    endif()

    find_path(SFML_INCLUDE_DIR
        NAMES SFML/Graphics.hpp
        PATH_SUFFIXES include
        PATHS ${_sfml_search_roots}
    )

    set(_sfml_lib_suffixes lib lib64)
    if(MSVC)
        list(PREPEND _sfml_lib_suffixes lib/Release)
    endif()

    find_library(SFML_GRAPHICS_LIBRARY
        NAMES sfml-graphics sfml-graphics-s sfml-graphics-d
        PATH_SUFFIXES ${_sfml_lib_suffixes}
        PATHS ${_sfml_search_roots}
    )
    find_library(SFML_WINDOW_LIBRARY
        NAMES sfml-window sfml-window-s sfml-window-d
        PATH_SUFFIXES ${_sfml_lib_suffixes}
        PATHS ${_sfml_search_roots}
    )
    find_library(SFML_SYSTEM_LIBRARY
        NAMES sfml-system sfml-system-s sfml-system-d
        PATH_SUFFIXES ${_sfml_lib_suffixes}
        PATHS ${_sfml_search_roots}
    )

    if(SFML_INCLUDE_DIR AND SFML_GRAPHICS_LIBRARY AND SFML_WINDOW_LIBRARY AND SFML_SYSTEM_LIBRARY)
        add_library(SFML::Graphics UNKNOWN IMPORTED)
        set_target_properties(SFML::Graphics PROPERTIES IMPORTED_LOCATION "${SFML_GRAPHICS_LIBRARY}")

        add_library(SFML::Window UNKNOWN IMPORTED)
        set_target_properties(SFML::Window PROPERTIES IMPORTED_LOCATION "${SFML_WINDOW_LIBRARY}")

        add_library(SFML::System UNKNOWN IMPORTED)
        set_target_properties(SFML::System PROPERTIES IMPORTED_LOCATION "${SFML_SYSTEM_LIBRARY}")

        set(SFML_FOUND TRUE)
    endif()
endif()

if(NOT SFML_FOUND)
    message(FATAL_ERROR "SFML not found. Use vcpkg/toolchain integration or set SFML_ROOT to a manual Windows SFML install.")
endif()

file(GLOB SRC_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB VENDOR_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/*.cpp")

add_executable(DroneNavReal
    main.cpp
    ${SRC_FILES}
    ${VENDOR_FILES}
)

set_target_properties(DroneNavReal PROPERTIES OUTPUT_NAME DroneNavReal)

target_include_directories(DroneNavReal PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor"
    ${SFML_INCLUDE_DIR}
)

target_link_libraries(DroneNavReal PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)