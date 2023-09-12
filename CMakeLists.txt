cmake_minimum_required(VERSION 3.26)
project(HesaiPythonWrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(HesaiLidar_SDK_2.0)

#configure_file(
#        "${CMAKE_CURRENT_SOURCE_DIR}/HesaiLidar_SDK_2.0/Version.h.in"
#        "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
#)

include_directories(
        HesaiLidar_SDK_2.0/libhesai
        HesaiLidar_SDK_2.0/libhesai/Lidar
        HesaiLidar_SDK_2.0/libhesai/UdpParser
        HesaiLidar_SDK_2.0/libhesai/UdpParser/include
        HesaiLidar_SDK_2.0/libhesai/UdpParser/src
        HesaiLidar_SDK_2.0/libhesai/UdpProtocol
        HesaiLidar_SDK_2.0/libhesai/Source/include
        HesaiLidar_SDK_2.0/libhesai/Container/include
        HesaiLidar_SDK_2.0/libhesai/Container/src
        HesaiLidar_SDK_2.0/libhesai/UdpParserGpu
        HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/include
        HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src
        HesaiLidar_SDK_2.0/libhesai/PtcClient/include
        HesaiLidar_SDK_2.0/libhesai/Logger/include
        HesaiLidar_SDK_2.0/libhesai/include
        HesaiLidar_SDK_2.0/driver
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(SWIG 4.0 COMPONENTS python)
include (UseSWIG)
set_property(SOURCE HesaiWrapper.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE HesaiWrapper.i PROPERTY GENERATED_INCLUDE_DIRECTORIES /usr/include/python3.10 ${CMAKE_SOURCE_DIR})
swig_add_library(${PROJECT_NAME}
                TYPE SHARED
                LANGUAGE python
                SOURCES HesaiWrapper.cpp HesaiWrapper.i
)

target_link_libraries(${PROJECT_NAME}
        source_lib
        container_lib
        ptcClient_lib
        log_lib
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES})