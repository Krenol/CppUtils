# CMakeList.txt : CMake project for RPiComponents, include source and define
# project specific logic here.

project (utils_test_project)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
 
set(TEST_PROJECT_SRCS 
${PROJECT_SOURCE_DIR}/test.cpp
)

include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")

# Add source to this project's executable.
add_executable (utils_test_project ${TEST_PROJECT_SRCS})

find_package(Threads REQUIRED)
find_package (Eigen3 REQUIRED)
find_library(pthread REQUIRED)
target_link_libraries(${PROJECT_NAME} pthread)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

target_link_libraries(utils_test_project utils)
