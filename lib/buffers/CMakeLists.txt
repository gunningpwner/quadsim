cmake_minimum_required(VERSION 3.10.2)
add_library(buffers_lib INTERFACE)
target_include_directories(buffers_lib INTERFACE include)

option(BUILD_BUFFER_TESTS "Build tests for the buffers module" ON)

if(BUILD_BUFFER_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()