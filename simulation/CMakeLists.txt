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


find_package(gz-transport15 REQUIRED)
find_package(gz-msgs15 REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED)

# Generate Inclination and Declination data
set(SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tools/generation")
set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/world_model/include")
set(WMM_HEADER "${GEN_DIR}/WMMData.h")
add_custom_command(
    OUTPUT ${WMM_HEADER}
    COMMAND Python3::Interpreter "${SCRIPTS_DIR}/WMM.py" 
            "${SCRIPTS_DIR}/WMM2025.COF" 
            "${WMM_HEADER}"
    DEPENDS "${SCRIPTS_DIR}/WMM.py" "${SCRIPTS_DIR}/WMM2025.COF"
    COMMENT "Generating WMM2025 Magnetic Lookup Table..."
)

add_custom_target(generate_wmm DEPENDS ${WMM_HEADER})

add_executable(gazebo_harness
    src/main.cpp
    src/TestHarness.cpp
    src/MotorInterface.cpp
    ../lib/flight_core/src/ESKF.cpp
    ../lib/flight_core/src/AutoLevelController.cpp
    ../lib/flight_core/src/PID.cpp
    ../lib/flight_core/src/MCE.cpp
)
add_dependencies(gazebo_harness generate_wmm)

target_compile_definitions(gazebo_harness PRIVATE GAZEBO) 

target_include_directories(gazebo_harness
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        src
        ../lib/flight_core/include
        ../lib/world_model/include
)
# Link the necessary libraries
target_link_libraries(gazebo_harness
    PRIVATE
        gz-transport14::gz-transport14
        gz-msgs11::gz-msgs11
        SDL2::SDL2
)