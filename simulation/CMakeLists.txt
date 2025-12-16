cmake_minimum_required(VERSION 3.10.2)
project(sim_harness)

if(NOT DEFINED ENV{WEBOTS_HOME})
    if(APPLE)
        set(WEBOTS_HOME "/Applications/Webots.app/Contents/Resources")
    elseif(WIN32)
        set(WEBOTS_HOME "C:/Program Files/Webots")
    else()
        set(WEBOTS_HOME "/usr/local/webots")
    endif()
else()
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
endif()

include_directories("${WEBOTS_HOME}/include/controller/cpp")
link_directories("${WEBOTS_HOME}/lib/controller")

# --- Dependency Management ---
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
)