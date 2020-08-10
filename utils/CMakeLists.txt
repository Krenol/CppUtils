project (utils)
cmake_minimum_required (VERSION 3.4)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(EIGEN_DIR ${PROJECT_SOURCE_DIR}/external/eigen)

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
        ${EIGEN_DIR}
)


find_package (Eigen3 REQUIRED NO_MODULE)
target_link_libraries (${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if(${BUILD_TEST})
    add_subdirectory(test)
endif()
