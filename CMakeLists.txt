cmake_minimum_required(VERSION 3.16)
project(pineapple_hardware_interface)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
## CANFD setup 
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_library(motor STATIC
    src/canfd/damiao.cpp 
)
target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/include
    ${LIBUSB_INCLUDE_DIRS}
)
target_link_libraries(motor PRIVATE
    Threads::Threads
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/lib/libu2canfd.a
    ${LIBUSB_LIBRARIES}
)

# FILE (GLOB HARDWARE_SRC
#         src/pineapple_hardware/*.cc
#         src/canfd/*.cc)

set(HARDWARE_DEPENDENCIES 
    pthread
    yaml-cpp
    unitree_sdk2)


add_executable(pineapple_hardware_interface  src/main.cc)
target_link_libraries(pineapple_hardware_interface  ${HARDWARE_DEPENDENCIES} motor)

SET(CMAKE_BUILD_TYPE Release)