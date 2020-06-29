# CMakeList.txt : CMake project for RPiComponents, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.4)
set(CMAKE_CXX_STANDARD 14)  # enable C++14 standard

project (utils)
 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
 
if (NOT "$ENV{BUILD_TEST}" STREQUAL "")
    set(BUILD_TEST "$ENV{BUILD_TEST}" CACHE INTERNAL "Copied from environment variable")
endif()
message("BUILD_TEST = ${BUILD_TEST}")

set(UTILS_PROJECT_SRCS
${PROJECT_SOURCE_DIR}/waiter.cpp
${PROJECT_SOURCE_DIR}/maths.cpp
)

 
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")
 
add_library(${PROJECT_NAME} SHARED ${UTILS_PROJECT_SRCS})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

if(${BUILD_TEST})
    add_subdirectory(test)
endif()