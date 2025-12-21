cmake_minimum_required(VERSION 3.10.2)
project(sim_harness)

set(CMAKE_CXX_STANDARD 17)

# --- 1. WEBOTS SETUP ---
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

# Include Webots Headers and Library
include_directories("${WEBOTS_HOME}/include/controller/cpp")
link_directories("${WEBOTS_HOME}/lib/controller")

# --- 2. DEPENDENCIES ---
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED)

# --- 3. GENERATE WMM (Keep your existing logic) ---
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

# --- 4. EXECUTABLE ---
add_executable(webots_harness
    src/main.cpp
    src/TestHarness.cpp
    src/MotorInterface.cpp
    ../lib/filters/src/ESKF.cpp
    ../lib/flight_core/src/PID.cpp
    ../lib/flight_core/src/MCE.cpp
)

add_dependencies(webots_harness generate_wmm)

# Use a specific define so you can #ifdef WEBOTS inside your code if needed
target_compile_definitions(webots_harness PRIVATE SIM)

target_include_directories(webots_harness
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        src
        ../lib/flight_core/include
        ../lib/world_model/include
        ../lib/buffers/include
        ../lib/filters/include
)

# Link against Webots Controller (and CppController wrapper)
target_link_libraries(webots_harness
    PRIVATE
    Controller
    CppController
)

# Fix for Linux/Mac to find the shared library at runtime
if(UNIX)
    set_target_properties(webots_harness PROPERTIES INSTALL_RPATH "${WEBOTS_HOME}/lib/controller")
endif()