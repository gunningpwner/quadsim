cmake_minimum_required(VERSION 3.10.2)
project(gazebo_subscriber)

# --- Dependency Management ---
include(FetchContent)

# Declare the serial library dependency from GitHub
FetchContent_Declare(
    serial
    GIT_REPOSITORY https://github.com/wjwwood/serial.git
    GIT_TAG        1.2.1 # Use a specific, stable tag for reproducibility
)

# This downloads the content but does NOT try to build it, avoiding the catkin dependency.
FetchContent_GetProperties(serial)
if(NOT serial_POPULATED)
    FetchContent_Populate(serial)
endif()

# The serial library is NOT header-only. We must compile its source files.
# We create a library target from the fetched source.
if(WIN32)
    set(SERIAL_SRC ${serial_SOURCE_DIR}/src/serial.cc ${serial_SOURCE_DIR}/src/impl/win.cc)
else()
    # Assuming a Unix-like system (Linux, macOS)
    set(SERIAL_SRC ${serial_SOURCE_DIR}/src/serial.cc ${serial_SOURCE_DIR}/src/impl/unix.cc)
endif()

add_library(serial_lib STATIC ${SERIAL_SRC})
# The library needs its own include directory to find its headers.
target_include_directories(serial_lib PUBLIC ${serial_SOURCE_DIR}/include)


find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(gazebo_link
    src/main.cpp
    src/TestHarness.cpp
    src/MotorInterface.cpp
    ../libs/flight_controller/ESKF.cpp
    ../libs/flight_controller/AutoLevelController.cpp
    ../libs/flight_controller/PID.cpp
    ../libs/flight_controller/coord_trans.cpp
    ../libs/flight_controller/MCE.cpp
)
target_compile_definitions(gazebo_link PRIVATE GAZEBO) 

target_include_directories(gazebo_link
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        src
        ../libs/common
        ../libs/flight_controller
)
# Link the necessary libraries
target_link_libraries(gazebo_link
    PRIVATE
        gz-transport14::gz-transport14
        gz-msgs11::gz-msgs11
        SDL2::SDL2
)