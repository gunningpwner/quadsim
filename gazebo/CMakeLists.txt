cmake_minimum_required(VERSION 3.10.2)
project(gazebo_subscriber)

# Find Gazebo Transport and Messages libraries
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)
# Add your executable
add_executable(gazebo_link
    src/main.cpp
    src/GazeboInterface.cpp
    src/ControllerInterface.cpp
    ../src/flight_controller/TruthFilter.cpp
    ../src/flight_controller/EKF.cpp
    ../src/flight_controller/BodyRateController.cpp
    ../src/common/PID.cpp
    ../src/common/coord_trans.cpp
)
target_include_directories(gazebo_link
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        src
        ../src/common
        ../src/flight_controller
)
# Link the necessary libraries
target_link_libraries(gazebo_link
    PRIVATE
        gz-transport14::gz-transport14
        gz-msgs11::gz-msgs11
        SDL2::SDL2
)