project (utils)
cmake_minimum_required (VERSION 3.4)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(BLAZE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/external/blaze)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(UTILS_PROJECT_SRCS
${PROJECT_SOURCE_DIR}/waiter.cpp
${PROJECT_SOURCE_DIR}/maths.cpp
${PROJECT_SOURCE_DIR}/environment.cpp
${PROJECT_SOURCE_DIR}/kalmann.cpp
)


add_library(${PROJECT_NAME} SHARED ${UTILS_PROJECT_SRCS})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

if(NOT TARGET blaze)
    add_subdirectory(${BLAZE_LIBRARY_PATH})
endif()
target_link_libraries(${PROJECT_NAME} blaze)

if(${BUILD_TEST})
    add_subdirectory(test)
endif()
