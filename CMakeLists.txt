cmake_minimum_required(VERSION 3.16)
project(DroneNavReal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SFML_DIR "C:/SFML/lib/cmake/SFML")

find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB VENDOR_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/*.cpp")

add_executable(DroneNavReal
    main.cpp
    ${SRC_FILES}
    ${VENDOR_FILES}
)

target_include_directories(DroneNavReal PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor"
)

target_link_libraries(DroneNavReal PRIVATE
    sfml-graphics
    sfml-window
    sfml-system
)