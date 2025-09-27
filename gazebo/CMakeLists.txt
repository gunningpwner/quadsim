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

set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/../.pio/libdeps/genericSTM32F405RG/MAVLink\ v2\ C\ library)
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(gazebo_link
    src/main.cpp
    src/GazeboInterface.cpp
    src/ControllerInterface.cpp
    ../libs/flight_controller/TruthFilter.cpp
    ../libs/flight_controller/EKF.cpp
    ../libs/flight_controller/MahonyFilter.cpp
    ../libs/flight_controller/BodyRateController.cpp
    ../libs/flight_controller/PID.cpp
    ../libs/flight_controller/coord_trans.cpp
)

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

add_executable(mavlink_bridge
    src/mavlink_bridge.cpp)
target_include_directories(mavlink_bridge
    PRIVATE
    ${MAVLINK_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mavlink_bridge
    PRIVATE
    gz-transport14::gz-transport14
    gz-msgs11::gz-msgs11
    serial_lib # Link against the serial library we just built
)

# The mavlink_bridge also needs the include path for the serial library.
target_include_directories(mavlink_bridge PRIVATE ${serial_SOURCE_DIR}/include)