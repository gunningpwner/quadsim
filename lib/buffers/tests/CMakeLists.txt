include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_ringbuffer test_ringbuffer.cpp)
if(MSVC)
    # Windows/Visual Studio flags
    target_compile_options(test_ringbuffer PRIVATE /Od /Zi)
else()
    # GCC/Clang (Linux/Mac) flags
    # -g: Add debug symbols
    # -O0: Disable optimizations (crucial for debugging)
    # -fno-inline: Prevent function inlining so stack traces make sense
    target_compile_options(test_ringbuffer PRIVATE -g -O0 -fno-inline)
endif()
target_link_libraries(test_ringbuffer GTest::gtest_main buffers_lib)
include(GoogleTest)
gtest_discover_tests(test_ringbuffer)